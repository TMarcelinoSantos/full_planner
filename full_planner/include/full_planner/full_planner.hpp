#ifndef FULL_PLANNER_HPP_
#define FULL_PLANNER_HPP_

#include "lart_msgs/msg/cone_array.hpp"
#include "lart_msgs/msg/path_array.hpp"

// Builds the track centerline from a cone map by Delaunay-triangulating the
// cones and connecting the midpoints of edges that join a blue cone to a
// yellow cone (i.e. the edges that cross the track between its boundaries).
class FullPlanner
{
public:
    FullPlanner() = default;

    lart_msgs::msg::PathArray computePath(const lart_msgs::msg::ConeArray & cone_map) const;
};

#endif  // FULL_PLANNER_HPP_
