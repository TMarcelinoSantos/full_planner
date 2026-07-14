#ifndef FULL_PLANNER_DELAUNAY_HPP_
#define FULL_PLANNER_DELAUNAY_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

// Minimal, self-contained 2D Delaunay triangulation (Bowyer-Watson).
// Kept dependency-free (no CGAL/boost) since it only needs to handle the
// handful of cones (tens to a few hundred) seen in a single track map.
namespace delaunay
{

struct Point
{
    double x;
    double y;
};

struct Triangle
{
    std::size_t a;
    std::size_t b;
    std::size_t c;
};

namespace detail
{

struct Edge
{
    std::size_t a;
    std::size_t b;

    bool operator==(const Edge & other) const
    {
        return (a == other.a && b == other.b) || (a == other.b && b == other.a);
    }
};

struct Circle
{
    Point center;
    double radius_sq;
};

inline Circle circumcircle(const Point & p1, const Point & p2, const Point & p3)
{
    const double ax = p1.x, ay = p1.y;
    const double bx = p2.x, by = p2.y;
    const double cx = p3.x, cy = p3.y;

    const double d = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));

    Circle circle;
    if (std::abs(d) < 1e-12) {
        // Degenerate (near-collinear) triangle: make the circle never match.
        circle.center = {0.0, 0.0};
        circle.radius_sq = -1.0;
        return circle;
    }

    const double a2 = ax * ax + ay * ay;
    const double b2 = bx * bx + by * by;
    const double c2 = cx * cx + cy * cy;

    const double ux = (a2 * (by - cy) + b2 * (cy - ay) + c2 * (ay - by)) / d;
    const double uy = (a2 * (cx - bx) + b2 * (ax - cx) + c2 * (bx - ax)) / d;

    circle.center = {ux, uy};
    const double dx = ax - ux;
    const double dy = ay - uy;
    circle.radius_sq = dx * dx + dy * dy;
    return circle;
}

}  // namespace detail

// Returns the Delaunay triangles for the given points, indexed into `points`.
inline std::vector<Triangle> triangulate(const std::vector<Point> & points)
{
    std::vector<Triangle> triangulation;
    const std::size_t n = points.size();
    if (n < 3) {
        return triangulation;
    }

    double min_x = points[0].x, max_x = points[0].x;
    double min_y = points[0].y, max_y = points[0].y;
    for (const auto & p : points) {
        min_x = std::min(min_x, p.x);
        max_x = std::max(max_x, p.x);
        min_y = std::min(min_y, p.y);
        max_y = std::max(max_y, p.y);
    }

    const double delta_max = std::max({max_x - min_x, max_y - min_y, 10.0});
    const double mid_x = (min_x + max_x) / 2.0;
    const double mid_y = (min_y + max_y) / 2.0;

    // Work with a super-triangle (indices n, n+1, n+2) large enough to
    // contain every input point, removed again at the end.
    std::vector<Point> work_points = points;
    work_points.push_back({mid_x - 20.0 * delta_max, mid_y - delta_max});
    work_points.push_back({mid_x, mid_y + 20.0 * delta_max});
    work_points.push_back({mid_x + 20.0 * delta_max, mid_y - delta_max});

    std::vector<Triangle> triangles;
    triangles.push_back({n, n + 1, n + 2});

    for (std::size_t pi = 0; pi < n; ++pi) {
        const Point & p = work_points[pi];

        std::vector<detail::Edge> polygon;
        std::vector<Triangle> kept_triangles;
        kept_triangles.reserve(triangles.size());

        for (const auto & tri : triangles) {
            const auto circle =
                detail::circumcircle(work_points[tri.a], work_points[tri.b], work_points[tri.c]);
            const double dx = p.x - circle.center.x;
            const double dy = p.y - circle.center.y;
            if (circle.radius_sq >= 0.0 && dx * dx + dy * dy < circle.radius_sq) {
                polygon.push_back({tri.a, tri.b});
                polygon.push_back({tri.b, tri.c});
                polygon.push_back({tri.c, tri.a});
            } else {
                kept_triangles.push_back(tri);
            }
        }

        // Edges shared by two removed ("bad") triangles are internal to the
        // cavity; only edges appearing once bound the re-triangulated hole.
        for (std::size_t i = 0; i < polygon.size(); ++i) {
            bool shared = false;
            for (std::size_t j = 0; j < polygon.size(); ++j) {
                if (i != j && polygon[i] == polygon[j]) {
                    shared = true;
                    break;
                }
            }
            if (!shared) {
                kept_triangles.push_back({polygon[i].a, polygon[i].b, pi});
            }
        }

        triangles = std::move(kept_triangles);
    }

    triangulation.reserve(triangles.size());
    for (const auto & tri : triangles) {
        if (tri.a < n && tri.b < n && tri.c < n) {
            triangulation.push_back(tri);
        }
    }

    return triangulation;
}

}  // namespace delaunay

#endif  // FULL_PLANNER_DELAUNAY_HPP_
