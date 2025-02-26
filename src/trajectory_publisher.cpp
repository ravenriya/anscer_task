#include <memory>
#include <chrono>
#include <string>
#include <vector>
#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "anscer_task/srv/save_trajectory.hpp"

using namespace std::chrono_literals;

class TrajectoryPublisher : public rclcpp::Node {
public:
    TrajectoryPublisher() : Node("trajectory_publisher") {
        // Create publisher for marker array
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "trajectory_markers", 10);

        // Create service for saving trajectory
        save_service_ = this->create_service<anscer_task::srv::SaveTrajectory>(
            "save_trajectory",
            std::bind(&TrajectoryPublisher::handleSaveTrajectory, this,
                     std::placeholders::_1, std::placeholders::_2));

        // Initialize TF buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Timer for collecting trajectory
        timer_ = this->create_wall_timer(
            100ms, std::bind(&TrajectoryPublisher::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "Trajectory Publisher node started");
    }

private:
    void timerCallback() {
        try {
            geometry_msgs::msg::TransformStamped transform = 
                tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);

            geometry_msgs::msg::PoseStamped pose;
            pose.header = transform.header;
            pose.pose.position.x = transform.transform.translation.x;
            pose.pose.position.y = transform.transform.translation.y;
            pose.pose.position.z = transform.transform.translation.z;
            pose.pose.orientation = transform.transform.rotation;

            trajectory_poses_.push_back(pose);
            publishTrajectoryMarkers();
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
        }
    }

    void publishTrajectoryMarkers() {
        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker line_strip;

        line_strip.header.frame_id = "map";
        line_strip.header.stamp = this->now();
        line_strip.ns = "trajectory";
        line_strip.id = 0;
        line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_strip.action = visualization_msgs::msg::Marker::ADD;
        line_strip.scale.x = 0.1;  // Line width
        line_strip.color.r = 1.0;
        line_strip.color.g = 0.0;
        line_strip.color.b = 0.0;
        line_strip.color.a = 1.0;
        line_strip.pose.orientation.w = 1.0;

        for (const auto& pose : trajectory_poses_) {
            geometry_msgs::msg::Point point;
            point.x = pose.pose.position.x;
            point.y = pose.pose.position.y;
            point.z = pose.pose.position.z;
            line_strip.points.push_back(point);
        }

        marker_array.markers.push_back(line_strip);
        marker_pub_->publish(marker_array);
    }

    void handleSaveTrajectory(
        const std::shared_ptr<anscer_task::srv::SaveTrajectory::Request> request,
        std::shared_ptr<anscer_task::srv::SaveTrajectory::Response> response) {
        
        if (trajectory_poses_.empty()) {
            response->success = false;
            response->message = "No trajectory data available";
            return;
        }

        size_t start_idx = 0;
        if (request->duration > 0.0) {
            auto current_time = this->now();
            for (size_t i = 0; i < trajectory_poses_.size(); ++i) {
                auto time_diff = current_time - trajectory_poses_[i].header.stamp;
                if (time_diff.seconds() <= request->duration) {
                    start_idx = i;
                    break;
                }
            }
        }

        std::ofstream file(request->filename);
        if (!file.is_open()) {
            response->success = false;
            response->message = "Could not open file for writing";
            return;
        }

        // Save as CSV format
        file << "timestamp,x,y,z,qx,qy,qz,qw\n";
        for (size_t i = start_idx; i < trajectory_poses_.size(); ++i) {
            const auto& pose = trajectory_poses_[i];
            file << pose.header.stamp.sec << "." << pose.header.stamp.nanosec << ","
                 << pose.pose.position.x << ","
                 << pose.pose.position.y << ","
                 << pose.pose.position.z << ","
                 << pose.pose.orientation.x << ","
                 << pose.pose.orientation.y << ","
                 << pose.pose.orientation.z << ","
                 << pose.pose.orientation.w << "\n";
        }

        file.close();
        response->success = true;
        response->message = "Trajectory saved successfully";
    }

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Service<anscer_task::srv::SaveTrajectory>::SharedPtr save_service_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::vector<geometry_msgs::msg::PoseStamped> trajectory_poses_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}