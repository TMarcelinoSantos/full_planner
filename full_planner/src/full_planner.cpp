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

    const std::vector<std::size_t> ordered = orderPath(midpoints, adjacency);
    if (ordered.size() < 2) {
        return path;
    }

    path.poses.reserve(ordered.size());
    path.curvature.reserve(ordered.size());
    path.distance.reserve(ordered.size());

    double cumulative_distance = 0.0;
    for (std::size_t k = 0; k < ordered.size(); ++k) {
        const delaunay::Point & p = midpoints[ordered[k]];

        double yaw;
        if (k + 1 < ordered.size()) {
            const delaunay::Point & next = midpoints[ordered[k + 1]];
            yaw = std::atan2(next.y - p.y, next.x - p.x);
        } else {
            const delaunay::Point & prev = midpoints[ordered[k - 1]];
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
            const delaunay::Point & prev = midpoints[ordered[k - 1]];
            cumulative_distance += std::hypot(p.x - prev.x, p.y - prev.y);
        }
        path.distance.push_back(static_cast<float>(cumulative_distance));

        if (k > 0 && k + 1 < ordered.size()) {
            path.curvature.push_back(curvatureAt(
                midpoints[ordered[k - 1]], midpoints[ordered[k]], midpoints[ordered[k + 1]]));
        } else {
            path.curvature.push_back(0.0f);
        }
    }

    return path;
}
