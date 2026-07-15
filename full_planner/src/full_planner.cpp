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

// Half-width (in resampled points) of the window used to estimate curvature
// at each path point: 4 points before it and 4 after, 9 in total including
// the point itself.
constexpr int kCurvatureHalfWindow = 4;

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

// Minimum/maximum radius (in metres) a circle fit is allowed to produce,
// same clamp used by ft_fsd_path_planning's circle_fit()-based curvature
// (fsd_path_planning/calculate_path/path_parameterization.py).
constexpr double kMinFitRadiusM = 1.0;
constexpr double kMaxFitRadiusM = 3000.0;

// Algebraically fits a circle to `points` using the "Hyper Fit" method
// (Al-Sharadqah & Chernov), ported line-for-line from circle_fit() in
// ft_fsd_path_planning's fsd_path_planning/utils/math_utils.py (itself
// adapted from the circle-fit PyPI package). Returns false (leaving the
// outputs untouched) if the fit is degenerate.
bool hyperFitCircle(const std::vector<delaunay::Point> & points, double & radius)
{
    const std::size_t n = points.size();
    if (n < 3) {
        return false;
    }

    double mean_x = 0.0, mean_y = 0.0;
    for (const auto & p : points) {
        mean_x += p.x;
        mean_y += p.y;
    }
    mean_x /= static_cast<double>(n);
    mean_y /= static_cast<double>(n);

    double mxy = 0.0, mxx = 0.0, myy = 0.0, mxz = 0.0, myz = 0.0, mzz = 0.0;
    for (const auto & p : points) {
        const double xi = p.x - mean_x;
        const double yi = p.y - mean_y;
        const double zi = xi * xi + yi * yi;
        mxy += xi * yi;
        mxx += xi * xi;
        myy += yi * yi;
        mxz += xi * zi;
        myz += yi * zi;
        mzz += zi * zi;
    }
    const double inv_n = 1.0 / static_cast<double>(n);
    mxy *= inv_n;
    mxx *= inv_n;
    myy *= inv_n;
    mxz *= inv_n;
    myz *= inv_n;
    mzz *= inv_n;

    const double mz = mxx + myy;
    const double cov_xy = mxx * myy - mxy * mxy;
    const double var_z = mzz - mz * mz;

    const double a2 = 4.0 * cov_xy - 3.0 * mz * mz - mzz;
    const double a1 = var_z * mz + 4.0 * cov_xy * mz - mxz * mxz - myz * myz;
    const double a0 = mxz * (mxz * myy - myz * mxy) + myz * (myz * mxx - mxz * mxy) - var_z * cov_xy;
    const double a22 = a2 + a2;

    // Newton iteration on the fit's characteristic polynomial root.
    double x = 0.0;
    double y = a0;
    constexpr int kMaxIter = 99;
    for (int iter = 0; iter < kMaxIter; ++iter) {
        const double dy = a1 + x * (a22 + 16.0 * x * x);
        if (dy == 0.0) {
            break;
        }
        const double x_new = x - y / dy;
        if (x_new == x || !std::isfinite(x_new)) {
            break;
        }
        const double y_new = a0 + x_new * (a1 + x_new * (a2 + 4.0 * x_new * x_new));
        if (std::abs(y_new) >= std::abs(y)) {
            break;
        }
        x = x_new;
        y = y_new;
    }

    const double det = x * x - x * mz + cov_xy;
    if (std::abs(det) < 1e-12) {
        return false;
    }

    const double center_x = (mxz * (myy - x) - myz * mxy) / det / 2.0;
    const double center_y = (myz * (mxx - x) - mxz * mxy) / det / 2.0;
    radius = std::sqrt(std::abs(center_x * center_x + center_y * center_y + mz));
    return true;
}

// Curvature at points[center] (1/radius, positive for a left turn),
// estimated from a window of up to half_window points on each side of it
// (2*half_window+1 in total, including the point itself). This mirrors the
// curvature principle used in ft_fsd_path_planning
// (calculate_path_curvature() in
// fsd_path_planning/calculate_path/path_parameterization.py): an algebraic
// circle is fit to the window via hyperFitCircle() above, its radius
// clamped to [kMinFitRadiusM, kMaxFitRadiusM], and the sign of the
// curvature (left vs. right turn) comes from the orientation (signed area)
// of the window's first, middle, and last points, rather than from the
// circle fit itself.
//
// For a closed loop, resampled.front() == resampled.back() (see the
// ring-closing step in computePath), so the window wraps cyclically
// through the distinct points [0, m) where m = points.size()-1; for an
// open path the window simply shrinks near the two real endpoints instead
// of wrapping.
float curvatureAtWindow(
    const std::vector<delaunay::Point> & points, std::size_t center, int half_window, bool is_closed_loop)
{
    std::vector<delaunay::Point> window;
    if (is_closed_loop && points.size() > 1) {
        const long long m = static_cast<long long>(points.size()) - 1;
        for (int offset = -half_window; offset <= half_window; ++offset) {
            const long long idx = ((static_cast<long long>(center) + offset) % m + m) % m;
            window.push_back(points[static_cast<std::size_t>(idx)]);
        }
    } else {
        const std::size_t n = points.size();
        const std::size_t lo = (center >= static_cast<std::size_t>(half_window)) ? center - half_window : 0;
        const std::size_t hi = std::min(n - 1, center + static_cast<std::size_t>(half_window));
        for (std::size_t idx = lo; idx <= hi; ++idx) {
            window.push_back(points[idx]);
        }
    }

    if (window.size() < 3) {
        return 0.0f;
    }

    double radius = 0.0;
    if (!hyperFitCircle(window, radius)) {
        return 0.0f;
    }
    radius = std::min(std::max(radius, kMinFitRadiusM), kMaxFitRadiusM);

    const delaunay::Point & p0 = window.front();
    const delaunay::Point & pm = window[window.size() / 2];
    const delaunay::Point & p1 = window.back();
    const double signed_area = (pm.x - p0.x) * (p1.y - p0.y) - (pm.y - p0.y) * (p1.x - p0.x);
    const double sign = (signed_area > 0.0) ? 1.0 : ((signed_area < 0.0) ? -1.0 : 0.0);

    return static_cast<float>(sign / radius);
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

lart_msgs::msg::PathArray FullPlanner::computePath(const lart_msgs::msg::ConeArray & cone_map) const
{
    lart_msgs::msg::PathArray path;
    path.header = cone_map.header;

    std::vector<delaunay::Point> cone_points;
    std::vector<std::size_t> boundary_cone_indices;
    cone_points.reserve(cone_map.cones.size());
    boundary_cone_indices.reserve(cone_map.cones.size());
    for (std::size_t i = 0; i < cone_map.cones.size(); ++i) {
        const auto class_type = cone_map.cones[i].class_type.data;
        if (class_type == lart_msgs::msg::Cone::BLUE || class_type == lart_msgs::msg::Cone::YELLOW) {
            cone_points.push_back(toPoint(cone_map.cones[i].position));
            boundary_cone_indices.push_back(i);
        }
    }
    if (cone_points.size() < 3) {
        return path;
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
        const bool i_is_blue =
            cone_map.cones[boundary_cone_indices[i]].class_type.data == lart_msgs::msg::Cone::BLUE;
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
            if (isTrackCrossingEdge(cone_map.cones[boundary_cone_indices[i]], cone_map.cones[boundary_cone_indices[j]])) {
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

    std::vector<delaunay::Point> smoothed = smoothPolyline(ordered_points, kSmoothingHalfWindow);

    // If every point has exactly two neighbours, the track has no
    // start/finish gap and the walk above traced a full ring - the "cut"
    // into a linear list still happened at some arbitrary point though, so
    // close the ring back up explicitly (repeat its first point at the
    // end) rather than leaving a seam there.
    const bool is_closed_loop = std::none_of(
        adjacency.begin(), adjacency.end(),
        [](const std::vector<std::size_t> & neighbors) { return neighbors.size() <= 1; });
    if (is_closed_loop) {
        smoothed.push_back(smoothed.front());
    }

    const std::vector<delaunay::Point> resampled = resampleAtSpacing(smoothed, kPathPointSpacingM);
    if (resampled.size() < 2) {
        return path;
    }

    path.points.reserve(resampled.size());

    double cumulative_distance = 0.0;
    for (std::size_t k = 0; k < resampled.size(); ++k) {
        const delaunay::Point & p = resampled[k];

        lart_msgs::msg::PathPoint point;
        point.x = static_cast<float>(p.x);
        point.y = static_cast<float>(p.y);

        if (k > 0) {
            const delaunay::Point & prev = resampled[k - 1];
            cumulative_distance += std::hypot(p.x - prev.x, p.y - prev.y);
        }
        point.distance = static_cast<float>(cumulative_distance);

        point.curvature = curvatureAtWindow(resampled, k, kCurvatureHalfWindow, is_closed_loop);

        // No velocity profile yet - left at the field's default (0).
        path.points.push_back(point);
    }

    return path;
}
