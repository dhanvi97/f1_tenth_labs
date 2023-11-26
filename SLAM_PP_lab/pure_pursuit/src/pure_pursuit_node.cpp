#include <sstream>
#include <string>
#include <cmath>
#include <vector>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
/// CHECK: include needed ROS msg type headers and libraries

using namespace std;
using std::placeholders::_1;


class PurePursuit : public rclcpp::Node
{
    // Implement PurePursuit
    // This is just a template, you are free to implement your own node!

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_subscription_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher;
    float lookahead_distance = 1.0;
    vector<tuple<float, float>> wp_locs;
    

public:
    PurePursuit() : Node("pure_pursuit_node")
    {
        // TODO: create ROS subscribers and publishers
        pose_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/pf/pose/odom", 1, std::bind(&PurePursuit::pose_callback, this, _1));
        drive_publisher = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);
        marker_publisher = this->create_publisher<visualization_msgs::msg::Marker>("marker", 10);
        path_publisher = this->create_publisher<nav_msgs::msg::Path>("path", 10);
        tuple<vector<float>, vector<float>, vector<float>, vector<float>> waypoints = readCSV("/home/ros_ws/src/pure_pursuit/data/wp.csv");
        vector<float> x_locs = get<0>(waypoints);
        vector<float> y_locs = get<1>(waypoints);
        for(int i=0; i < x_locs.size(); i++){
            this->wp_locs.push_back(make_tuple(x_locs[i], y_locs[i]));
        }
    }

    void pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg)
    {
        // TODO: find the current waypoint to track using methods mentioned in lecture
        geometry_msgs::msg::Pose pose = pose_msg->pose.pose;
        int closest_point_index = closest_point_on_waypoints(pose);
        publish_marker(closest_point_index);
        geometry_msgs::msg::Pose closest_point_pose;
        closest_point_pose.position.x = get<0>(this->wp_locs[closest_point_index]);
        closest_point_pose.position.y = get<1>(this->wp_locs[closest_point_index]);
        closest_point_pose.position.z = 0;
        closest_point_pose.orientation.x = 0;
        closest_point_pose.orientation.y = 0;
        closest_point_pose.orientation.z = 0;
        closest_point_pose.orientation.w = 1;
        geometry_msgs::msg::Pose goal_pose = transform_to_vehicle_frame(closest_point_pose, pose);
        RCLCPP_INFO(this->get_logger(), "delta pose (%f, %f)", goal_pose.position.x, goal_pose.position.y);

        // TODO: transform goal point to vehicle frame of reference


        // TODO: calculate curvature/steering angle
        float steering_angle = 2*goal_pose.position.y/(pow(this->lookahead_distance, 2));
        RCLCPP_INFO(this->get_logger(), "steering angle %f", steering_angle);
        if(abs(steering_angle) > 0.4189){
            steering_angle = 0.4189*steering_angle/abs(steering_angle);
        }
        // TODO: publish drive message, don't forget to limit the steering angle.
        ackermann_msgs::msg::AckermannDriveStamped drive_msg;
        drive_msg.header.stamp = this->now();
        drive_msg.drive.speed = 1.0;
        drive_msg.drive.steering_angle = steering_angle;
        this->drive_publisher->publish(drive_msg);
        publish_path();

    }

    geometry_msgs::msg::Pose transform_to_vehicle_frame(geometry_msgs::msg::Pose goal_pose, geometry_msgs::msg::Pose robot_pose){
        float x = goal_pose.position.x - robot_pose.position.x;
        float y = goal_pose.position.y - robot_pose.position.y;
        tf2::Quaternion q(robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        float theta = atan2(y, x);
        float del_theta = theta - yaw;
        geometry_msgs::msg::Pose transformed_pose;
        transformed_pose.position.x = sqrt(pow(x,2) + pow(y,2))*cos(del_theta);
        transformed_pose.position.y = sqrt(pow(x,2) + pow(y,2))*sin(del_theta);
        transformed_pose.orientation = goal_pose.orientation;
        return transformed_pose;
    }

    void publish_marker(int closest_point_index){
        float x = get<0>(this->wp_locs[closest_point_index]);
        float y = get<1>(this->wp_locs[closest_point_index]);

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "my_namespace";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = -0.707;
        marker.pose.orientation.w = 0.707;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        this->marker_publisher->publish(marker);
    }

    // Find closest point at lookahead distance on path from current robot pose
    int closest_point_on_waypoints(geometry_msgs::msg::Pose robot_pose){
        float min_dist = 1000000;
        float best_index = -1;
        int min_index = -1;
        for(int i=0; i < this->wp_locs.size(); i++){
            float dist = sqrt(pow(get<0>(this->wp_locs[i]) - robot_pose.position.x, 2) + pow(get<1>(this->wp_locs[i]) - robot_pose.position.y, 2));
            if(dist <= this->lookahead_distance){
                best_index = i;
            }
            if(dist < min_dist){
                min_dist = dist;
                min_index = i;
            }
        }
        if (best_index != -1){
            return best_index;
        } else {
        return min_index;
        }
    }

    // Function to read 4 column csv file and return 4 vectors
    std::tuple<std::vector<float>, std::vector<float>, std::vector<float>, std::vector<float>> readCSV(const std::string& filename) {
        std::vector<float> column1, column2, column3, column4;

        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Failed to open the file: " << filename << std::endl;
            return std::make_tuple(column1, column2, column3, column4);
        }

        std::string line;
        while (std::getline(file, line)) {
            std::istringstream lineStream(line);
            std::string cell;
            std::vector<std::string> tokens;

            while (std::getline(lineStream, cell, ',')) {
                tokens.push_back(cell);
            }

            if (tokens.size() == 4) {
                column1.push_back(std::stof(tokens[0]));
                column2.push_back(std::stof(tokens[1]));
                column3.push_back(std::stof(tokens[2]));
                column4.push_back(std::stof(tokens[3]));
            }
        }

        file.close();

        return std::make_tuple(column1, column2, column3, column4);
    }

    void publish_path(){
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "map";
        for(int i=0; i < this->wp_locs.size(); i++){
            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = this->now();
            pose.header.frame_id = "map";
            pose.pose.position.x = get<0>(this->wp_locs[i]);
            pose.pose.position.y = get<1>(this->wp_locs[i]);
            path_msg.poses.push_back(pose);
        }
        this->path_publisher->publish(path_msg);
    }
    ~PurePursuit() {}
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuit>());
    rclcpp::shutdown();
    return 0;
}