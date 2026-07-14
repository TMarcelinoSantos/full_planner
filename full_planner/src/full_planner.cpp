#include "full_planner/full_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <utility>
#include <vector>

#include "full_planner/delaunay.hpp"

namespace
{

constexpr std::size_t kInvalidIndex = std::numeric_limits<std::size_t>::max();

// Target point density: 100 points per 20 m of path.
constexpr double kPathPointSpacingM = 20.0 / 100.0;

// Half-width (in points) of the moving-average smoothing window applied to
// the raw Delaunay-edge midpoints before resampling, to smooth out the
// zig-zag those midpoints otherwise have from cone-to-cone jitter.
constexpr int kSmoothingHalfWindow = 2;

delaunay::Point toPoint(const geometry_msgs::msg::Point & p)
{
    return {p.x, p.y};
}

// An edge "crosses" the track (and therefore its midpoint lies on the
// centerline) when it joins one blue cone to one yellow cone.
bool isTrackCrossingEdge(const lart_msgs::msg::Cone & a, const lart_msgs::msg::Cone & b)
{
    const auto ca = a.class_type.data;
    const auto cb = b.class_type.data;
    return (ca == lart_msgs::msg::Cone::YELLOW && cb == lart_msgs::msg::Cone::BLUE) ||
           (ca == lart_msgs::msg::Cone::BLUE && cb == lart_msgs::msg::Cone::YELLOW);
}

geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw)
{
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw / 2.0);
    q.w = std::cos(yaw / 2.0);
    return q;
}

// Signed Menger curvature of the three points (1/radius of the circle
// through them, positive for a left turn).
float curvatureAt(const delaunay::Point & p0, const delaunay::Point & p1, const delaunay::Point & p2)
{
    const double a = std::hypot(p1.x - p0.x, p1.y - p0.y);
    const double b = std::hypot(p2.x - p1.x, p2.y - p1.y);
    const double c = std::hypot(p2.x - p0.x, p2.y - p0.y);
    const double cross = (p1.x - p0.x) * (p2.y - p0.y) - (p1.y - p0.y) * (p2.x - p0.x);
    const double denom = a * b * c;
    if (denom < 1e-9) {
        return 0.0f;
    }
    return static_cast<float>(2.0 * cross / denom);
}

// Walks the midpoint adjacency graph into a single ordered sequence covering
// the whole track. Away from the start/finish gap (where the orange cones
// break the blue/yellow alternation) the graph is a simple chain - or a
// closed loop if there is no gap - so a straightest-continuation walk
// threads it correctly even where a handful of triangles create a stray
// branch. Starting from a degree-<=1 endpoint when one exists is what
// guarantees the walk covers the full chain in one direction, rather than
// risking its first step onto a short dead-end stub.
std::vector<std::size_t> orderPath(
    const std::vector<delaunay::Point> & midpoints,
    const std::vector<std::vector<std::size_t>> & adjacency)
{
    const std::size_t n = midpoints.size();

    std::size_t start = 0;
    for (std::size_t i = 0; i < n; ++i) {
        if (adjacency[i].size() <= 1) {
            start = i;
            break;
        }
    }

    std::vector<bool> visited(n, false);
    std::vector<std::size_t> ordered;
    ordered.reserve(n);

    std::size_t current = start;
    std::size_t previous = kInvalidIndex;
    visited[current] = true;
    ordered.push_back(current);

    while (true) {
        double ref_dx = 0.0;
        double ref_dy = 0.0;
        if (previous != kInvalidIndex) {
            ref_dx = midpoints[current].x - midpoints[previous].x;
            ref_dy = midpoints[current].y - midpoints[previous].y;
        }
        const double ref_norm = std::hypot(ref_dx, ref_dy);

        std::size_t next = kInvalidIndex;
        double best_score = -std::numeric_limits<double>::infinity();
        for (const std::size_t candidate : adjacency[current]) {
            if (visited[candidate]) {
                continue;
            }
            double score = 0.0;
            if (ref_norm > 1e-9) {
                const double dx = midpoints[candidate].x - midpoints[current].x;
                const double dy = midpoints[candidate].y - midpoints[current].y;
                const double norm = std::hypot(dx, dy);
                if (norm > 1e-9) {
                    score = (dx * ref_dx + dy * ref_dy) / (norm * ref_norm);
                }
            }
            if (score > best_score) {
                best_score = score;
                next = candidate;
            }
        }

        if (next == kInvalidIndex) {
            break;
        }
        previous = current;
        current = next;
        visited[current] = true;
        ordered.push_back(current);
    }

    return ordered;
}

// FSD convention: driving forward, the blue boundary is on the left and the
// yellow boundary is on the right. For a track-crossing edge, the vector
// from the yellow cone to the blue cone points from the right boundary to
// the left one, i.e. it *is* the left-hand direction relative to travel;
// rotating it -90 degrees (clockwise) therefore gives the forward direction
// consistent with "blue on the left".
delaunay::Point forwardDirectionForBlueOnLeft(const delaunay::Point & blue, const delaunay::Point & yellow)
{
    const double vx = blue.x - yellow.x;
    const double vy = blue.y - yellow.y;
    return {vy, -vx};
}

// Orients the ordered midpoint sequence so blue cones end up on the left of
// the direction of travel: each midpoint knows the specific blue/yellow
// cone pair that produced it, giving a local "forward" estimate to compare
// against the actual walk direction there. orderPath() has no colour
// information (it only follows adjacency), so it may equally well have
// walked the chain in the opposite direction; a majority vote over all
// interior points decides whether to reverse it.
bool shouldReverseForBlueOnLeft(
    const std::vector<std::size_t> & ordered,
    const std::vector<delaunay::Point> & midpoints,
    const std::vector<delaunay::Point> & blue_positions,
    const std::vector<delaunay::Point> & yellow_positions)
{
    int votes = 0;
    for (std::size_t k = 1; k + 1 < ordered.size(); ++k) {
        const delaunay::Point & prev = midpoints[ordered[k - 1]];
        const delaunay::Point & next = midpoints[ordered[k + 1]];
        const double travel_x = next.x - prev.x;
        const double travel_y = next.y - prev.y;

        const delaunay::Point forward =
            forwardDirectionForBlueOnLeft(blue_positions[ordered[k]], yellow_positions[ordered[k]]);

        const double dot = travel_x * forward.x + travel_y * forward.y;
        if (dot > 0.0) {
            ++votes;
        } else if (dot < 0.0) {
            --votes;
        }
    }
    return votes < 0;
}

// Centered moving average over the point sequence (clamped at the ends, so
// endpoints are smoothed with a shrinking window rather than wrapped
// around - the path is treated as open even when the track is a closed
// loop, since the start/finish gap already breaks it there).
std::vector<delaunay::Point> smoothPolyline(const std::vector<delaunay::Point> & points, int half_window)
{
    const std::size_t n = points.size();
    std::vector<delaunay::Point> smoothed(n);
    for (std::size_t i = 0; i < n; ++i) {
        const std::size_t lo = (i >= static_cast<std::size_t>(half_window)) ? i - half_window : 0;
        const std::size_t hi = std::min(n - 1, i + static_cast<std::size_t>(half_window));

        double sum_x = 0.0;
        double sum_y = 0.0;
        for (std::size_t j = lo; j <= hi; ++j) {
            sum_x += points[j].x;
            sum_y += points[j].y;
        }
        const double count = static_cast<double>(hi - lo + 1);
        smoothed[i] = {sum_x / count, sum_y / count};
    }
    return smoothed;
}

// Resamples the polyline to uniform arc-length spacing via linear
// interpolation between the (already smoothed) input points, always
// including the original last point so the resampled path still reaches
// the end of the track.
std::vector<delaunay::Point> resampleAtSpacing(const std::vector<delaunay::Point> & points, double spacing)
{
    if (points.size() < 2) {
        return points;
    }

    std::vector<double> cumulative(points.size(), 0.0);
    for (std::size_t i = 1; i < points.size(); ++i) {
        cumulative[i] = cumulative[i - 1] + std::hypot(points[i].x - points[i - 1].x, points[i].y - points[i - 1].y);
    }
    const double total_length = cumulative.back();

    std::vector<delaunay::Point> resampled;
    std::size_t seg = 0;
    for (double target = 0.0; target < total_length; target += spacing) {
        while (seg + 2 < points.size() && cumulative[seg + 1] < target) {
            ++seg;
        }
        const double seg_len = cumulative[seg + 1] - cumulative[seg];
        const double t = seg_len > 1e-9 ? (target - cumulative[seg]) / seg_len : 0.0;
        resampled.push_back({
            points[seg].x + t * (points[seg + 1].x - points[seg].x),
            points[seg].y + t * (points[seg + 1].y - points[seg].y),
        });
    }
    resampled.push_back(points.back());

    return resampled;
}

}  // namespace

lart_msgs::msg::PathSpline FullPlanner::computePath(const lart_msgs::msg::ConeArray & cone_map) const
{
    lart_msgs::msg::PathSpline path;
    path.header = cone_map.header;

    const std::size_t cone_count = cone_map.cones.size();
    if (cone_count < 3) {
        return path;
    }

    std::vector<delaunay::Point> cone_points;
    cone_points.reserve(cone_count);
    for (const auto & cone : cone_map.cones) {
        cone_points.push_back(toPoint(cone.position));
    }

    const auto triangles = delaunay::triangulate(cone_points);

    // Deduplicated midpoints of track-crossing edges (an edge is shared by
    // up to two triangles) plus the graph linking midpoints that belong to
    // the same triangle - that's how the centerline threads between
    // consecutive triangles across the track.
    std::vector<delaunay::Point> midpoints;
    std::vector<delaunay::Point> blue_positions;
    std::vector<delaunay::Point> yellow_positions;
    std::vector<std::vector<std::size_t>> adjacency;
    std::map<std::pair<std::size_t, std::size_t>, std::size_t> edge_to_midpoint;

    auto midpointIdForEdge = [&](std::size_t i, std::size_t j) -> std::size_t {
        const auto key = std::minmax(i, j);
        const auto it = edge_to_midpoint.find(key);
        if (it != edge_to_midpoint.end()) {
            return it->second;
        }
        const std::size_t id = midpoints.size();
        midpoints.push_back({
            (cone_points[i].x + cone_points[j].x) / 2.0,
            (cone_points[i].y + cone_points[j].y) / 2.0,
        });
        const bool i_is_blue = cone_map.cones[i].class_type.data == lart_msgs::msg::Cone::BLUE;
        blue_positions.push_back(i_is_blue ? cone_points[i] : cone_points[j]);
        yellow_positions.push_back(i_is_blue ? cone_points[j] : cone_points[i]);
        adjacency.emplace_back();
        edge_to_midpoint.emplace(key, id);
        return id;
    };

    for (const auto & tri : triangles) {
        const std::size_t verts[3] = {tri.a, tri.b, tri.c};
        std::vector<std::size_t> crossing_midpoints;
        for (int e = 0; e < 3; ++e) {
            const std::size_t i = verts[e];
            const std::size_t j = verts[(e + 1) % 3];
            if (isTrackCrossingEdge(cone_map.cones[i], cone_map.cones[j])) {
                crossing_midpoints.push_back(midpointIdForEdge(i, j));
            }
        }
        // With only two boundary colors, a triangle has at most two
        // track-crossing edges; link their midpoints to continue the path
        // through the triangle. (0 or 1 crossing edges: nothing to link.)
        if (crossing_midpoints.size() == 2) {
            adjacency[crossing_midpoints[0]].push_back(crossing_midpoints[1]);
            adjacency[crossing_midpoints[1]].push_back(crossing_midpoints[0]);
        }
    }

    if (midpoints.size() < 2) {
        return path;
    }

    std::vector<std::size_t> ordered = orderPath(midpoints, adjacency);
    if (ordered.size() < 2) {
        return path;
    }

    if (shouldReverseForBlueOnLeft(ordered, midpoints, blue_positions, yellow_positions)) {
        std::reverse(ordered.begin(), ordered.end());
    }

    std::vector<delaunay::Point> ordered_points;
    ordered_points.reserve(ordered.size());
    for (const std::size_t idx : ordered) {
        ordered_points.push_back(midpoints[idx]);
    }

    const std::vector<delaunay::Point> smoothed = smoothPolyline(ordered_points, kSmoothingHalfWindow);
    const std::vector<delaunay::Point> resampled = resampleAtSpacing(smoothed, kPathPointSpacingM);
    if (resampled.size() < 2) {
        return path;
    }

    path.poses.reserve(resampled.size());
    path.curvature.reserve(resampled.size());
    path.distance.reserve(resampled.size());

    double cumulative_distance = 0.0;
    for (std::size_t k = 0; k < resampled.size(); ++k) {
        const delaunay::Point & p = resampled[k];

        double yaw;
        if (k + 1 < resampled.size()) {
            const delaunay::Point & next = resampled[k + 1];
            yaw = std::atan2(next.y - p.y, next.x - p.x);
        } else {
            const delaunay::Point & prev = resampled[k - 1];
            yaw = std::atan2(p.y - prev.y, p.x - prev.x);
        }

        geometry_msgs::msg::PoseStamped pose;
        pose.header = cone_map.header;
        pose.pose.position.x = p.x;
        pose.pose.position.y = p.y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation = quaternionFromYaw(yaw);
        path.poses.push_back(pose);

        if (k > 0) {
            const delaunay::Point & prev = resampled[k - 1];
            cumulative_distance += std::hypot(p.x - prev.x, p.y - prev.y);
        }
        path.distance.push_back(static_cast<float>(cumulative_distance));

        if (k > 0 && k + 1 < resampled.size()) {
            path.curvature.push_back(curvatureAt(resampled[k - 1], resampled[k], resampled[k + 1]));
        } else {
            path.curvature.push_back(0.0f);
        }
    }

    return path;
}
