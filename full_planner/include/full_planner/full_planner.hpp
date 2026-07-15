#ifndef FULL_PLANNER_HPP_
#define FULL_PLANNER_HPP_

#include <cstddef>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "lart_msgs/msg/cone_array.hpp"
#include "lart_msgs/msg/path_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

// Builds the track centerline from a cone map by Delaunay-triangulating the
// cones and connecting the midpoints of edges that join a blue cone to a
// yellow cone (i.e. the edges that cross the track between its boundaries).
class FullPlanner
{
public:
    FullPlanner() = default;

    lart_msgs::msg::PathArray computePath(const lart_msgs::msg::ConeArray & cone_map) const;

    // Slice of full_path starting at the point closest to current_pose and
    // containing up to horizon_points points, wrapping past the
    // start/finish line for closed-loop tracks.
    lart_msgs::msg::PathArray computeHorizon(
        const lart_msgs::msg::PathArray & full_path,
        const geometry_msgs::msg::PoseStamped & current_pose,
        std::size_t horizon_points) const;

    // Red line-strip marker tracing the given path, for visualizing e.g. the
    // published horizon slice in rviz/foxglove.
    visualization_msgs::msg::MarkerArray buildPathMarker(const lart_msgs::msg::PathArray & path) const;
};

#endif  // FULL_PLANNER_HPP_
