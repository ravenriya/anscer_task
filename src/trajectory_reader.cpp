#include <memory>
#include <chrono>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

class TrajectoryReader : public rclcpp::Node {
public:
    TrajectoryReader() : Node("trajectory_reader") {
        // Declare parameters
        this->declare_parameter("trajectory_file", "trajectory.csv");
        this->declare_parameter("publish_rate", 1.0);  // Hz

        // Create publisher for marker array
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "trajectory_visualization", 10);

        // Initialize TF buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Read trajectory file
        readTrajectoryFile();

        // Create timer for publishing
        double publish_rate = this->get_parameter("publish_rate").as_double();
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0/publish_rate),
            std::bind(&TrajectoryReader::publishTrajectory, this));

        RCLCPP_INFO(this->get_logger(), "Trajectory Reader node started");
    }

private:
    void readTrajectoryFile() {
        std::string filename = this->get_parameter("trajectory_file").as_string();
        std::ifstream file(filename);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open file: %s", filename.c_str());
            return;
        }

        trajectory_poses_.clear();
        std::string line;
        // Skip header line
        std::getline(file, line);

        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string value;
            std::vector<double> values;

            while (std::getline(ss, value, ',')) {
                values.push_back(std::stod(value));
            }

            if (values.size() == 8) {  // timestamp + x,y,z + qx,qy,qz,qw
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = "map";
                
                // Set timestamp
                int32_t sec = static_cast<int32_t>(values[0]);
                uint32_t nanosec = static_cast<uint32_t>((values[0] - sec) * 1e9);
                pose.header.stamp = rclcpp::Time(sec, nanosec);

                pose.pose.position.x = values[1];
                pose.pose.position.y = values[2];
                pose.pose.position.z = values[3];
                pose.pose.orientation.x = values[4];
                pose.pose.orientation.y = values[5];
                pose.pose.orientation.z = values[6];
                pose.pose.orientation.w = values[7];

                trajectory_poses_.push_back(pose);
            }
        }
        RCLCPP_INFO(this->get_logger(), "Loaded %zu trajectory points", trajectory_poses_.size());
    }

    void publishTrajectory() {
        if (trajectory_poses_.empty()) {
            return;
        }

        try {
            visualization_msgs::msg::MarkerArray marker_array;
            visualization_msgs::msg::Marker line_strip;

            line_strip.header.frame_id = "odom";  // Publishing in odom frame
            line_strip.header.stamp = this->now();
            line_strip.ns = "trajectory_visualization";
            line_strip.id = 0;
            line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line_strip.action = visualization_msgs::msg::Marker::ADD;
            line_strip.scale.x = 0.1;  // Line width
            line_strip.color.r = 0.0;
            line_strip.color.g = 0.0;
            line_strip.color.b = 1.0;  // Blue color for loaded trajectory
            line_strip.color.a = 1.0;
            line_strip.pose.orientation.w = 1.0;

            // Transform each point to odom frame
            for (const auto& pose : trajectory_poses_) {
                geometry_msgs::msg::PoseStamped transformed_pose;
                
                // Try to transform from map to odom
                try {
                    geometry_msgs::msg::TransformStamped transform = 
                        tf_buffer_->lookupTransform("odom", "map", tf2::TimePointZero);
                    
                    tf2::doTransform(pose, transformed_pose, transform);

                    geometry_msgs::msg::Point point;
                    point.x = transformed_pose.pose.position.x;
                    point.y = transformed_pose.pose.position.y;
                    point.z = transformed_pose.pose.position.z;
                    line_strip.points.push_back(point);
                }
                catch (tf2::TransformException &ex) {
                    RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
                    return;
                }
            }

            marker_array.markers.push_back(line_strip);
            marker_pub_->publish(marker_array);
        }
        catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error publishing trajectory: %s", e.what());
        }
    }

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::vector<geometry_msgs::msg::PoseStamped> trajectory_poses_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryReader>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}