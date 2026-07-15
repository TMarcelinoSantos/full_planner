#ifndef FULL_PLANNER_NODE_H_
#define FULL_PLANNER_NODE_H_


/*------------------------------------------------------------------------------*/
/*                                   INCLUDES                                   */
/*------------------------------------------------------------------------------*/

#include <string>

#include "lart_common.h"
#include "topics.h"
#include "lart_msgs/msg/cone_array.hpp"
#include "lart_msgs/msg/path_array.hpp"
#include "lart_msgs/msg/slam_stats.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "full_planner/full_planner.hpp"

class PlannerNode : public rclcpp::Node
{
public:
    PlannerNode();

private:

protected:
    void slamMapCallback(const lart_msgs::msg::ConeArray::SharedPtr msg);
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void lapCallback(const lart_msgs::msg::SlamStats::SharedPtr msg);
    // Dumps the received cone map and the computed path to cones.csv and
    // path.csv under csv_output_dir_, for offline inspection with
    // scripts/plot_path.py.
    void writeConesCsv(const lart_msgs::msg::ConeArray & cone_map) const;
    void writePathCsv(const lart_msgs::msg::PathArray & path) const;

    rclcpp::Subscription<lart_msgs::msg::ConeArray>::SharedPtr slam_map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<lart_msgs::msg::SlamStats>::SharedPtr lap_sub_;
    rclcpp::Publisher<lart_msgs::msg::PathArray>::SharedPtr final_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_marker_pub_;

    FullPlanner full_planner_;
    lart_msgs::msg::PathArray full_final_path;
    bool map_completed_ = false;
    bool path_calculated_ = false;
    std::string csv_output_dir_;
    std::size_t horizon_points_ = 100;
};

#endif