#include "full_planner/planner_node.hpp"

#include <fstream>

using std::placeholders::_1;

PlannerNode::PlannerNode(): Node("full_planner_node")
{
    csv_output_dir_ = this->declare_parameter<std::string>("csv_output_dir", "/home/tomas/data/ros2_ws");

    slam_map_sub_ = this->create_subscription<lart_msgs::msg::ConeArray>(
        TOPIC_MAP, 10, std::bind(&PlannerNode::slamMapCallback, this, _1));

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        TOPIC_SLAM_POSE, 10, std::bind(&PlannerNode::poseCallback, this, _1));

    lap_sub_ = this->create_subscription<lart_msgs::msg::SlamStats>(
        TOPIC_STATS, 10, std::bind(&PlannerNode::lapCallback, this, _1)
    );

    path_pub_ = this->create_publisher<lart_msgs::msg::PathArray>(TOPIC_PATH, 10);
}

void PlannerNode::lapCallback(const lart_msgs::msg::SlamStats::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received lap stats: lap_count=%d",
                msg->lap_count);

    if(msg->lap_count > 0) {
        this->map_completed_ = true;
    }
}

void PlannerNode::slamMapCallback(const lart_msgs::msg::ConeArray::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received SLAM map with %zu cones", msg->cones.size());

    if(this->path_calculated_){
        RCLCPP_INFO(this->get_logger(), "Path has already been calculated, ignoring new map");
        return;
    }

    if(!this->map_completed_) {
        RCLCPP_INFO(this->get_logger(), "Map isn't completed yet, waiting for lap completion before computing path");
        return;
    }

    const lart_msgs::msg::PathArray path = full_planner_.computePath(*msg);
    if (path.points.empty()) {
        RCLCPP_WARN(this->get_logger(), "Could not build a midline path from the received cone map");
        return;
    }

    this->path_calculated_ = true;
    //path_pub_->publish(path);

    writeConesCsv(*msg);
    writePathCsv(path);
}

void PlannerNode::writeConesCsv(const lart_msgs::msg::ConeArray & cone_map) const
{
    const std::string filepath = csv_output_dir_ + "/cones.csv";
    std::ofstream file(filepath);
    if (!file.is_open()) {
        RCLCPP_WARN(this->get_logger(), "Could not open '%s' for writing", filepath.c_str());
        return;
    }

    file << "x,y,class_type,cone_id\n";
    for (const auto & cone : cone_map.cones) {
        file << cone.position.x << ',' << cone.position.y << ','
             << cone.class_type.data << ',' << cone.cone_id.data << '\n';
    }
    RCLCPP_INFO(this->get_logger(), "Wrote %zu cones to '%s'", cone_map.cones.size(), filepath.c_str());
}

void PlannerNode::writePathCsv(const lart_msgs::msg::PathArray & path) const
{
    const std::string filepath = csv_output_dir_ + "/path.csv";
    std::ofstream file(filepath);
    if (!file.is_open()) {
        RCLCPP_WARN(this->get_logger(), "Could not open '%s' for writing", filepath.c_str());
        return;
    }

    file << "index,x,y,distance,curvature,velocity\n";
    for (std::size_t i = 0; i < path.points.size(); ++i) {
        const auto & p = path.points[i];
        file << i << ',' << p.x << ',' << p.y << ','
             << p.distance << ',' << p.curvature << ',' << p.velocity << '\n';
    }
    RCLCPP_INFO(this->get_logger(), "Wrote %zu path points to '%s'", path.points.size(), filepath.c_str());
}

void PlannerNode::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received pose: [x: %f, y: %f]",
                msg->pose.position.x, msg->pose.position.y);

    latest_pose_ = *msg;
    has_pose_ = true;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlannerNode>());
    rclcpp::shutdown();
    return 0;
}