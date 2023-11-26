#include <fstream>
#include <string>
#include <cmath>
#include <iostream>
#include <vector>
#include <tuple>
#include <sstream>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <yaml-cpp/yaml.h>
using namespace std;

// Car state class containing x, y, theta, v
class CarState{
  public:
    // Car x position in map frame
    float x;
    // Car y position in map frame
    float y;
    // Car heading angle
    float theta;
    // Car velocity in m/s
    float v;
    // Constructor for CarState given x, y, theta, v
    CarState(float x, float y, float theta, float v);
    // Default Constructor setting all values to 0
    CarState();  
};

// Config class containing all the parameters for the MPC controller
class Config{
  public:
    // Lookahead horizon in number of timesteps
    int N;
    // Time step
    float dt;
    // Distance step along path 
    float dtk;
    // Wheelbase distance for car
    float wheelbase;
    float v_max, v_min, delta_max, delta_min;
    float Q[3], R[2], QN[3];

    // Operation parameters
    std::string operation_mode;
    std::string path_source;
    std::string path_filename;
    std::string path_topic;
    // Constructor for Config given all the parameters
    Config(float dt, float dtk, int N, float Q[3], float R[2], float QN[3], float v_max, float v_min, float delta_max, float delta_min, float wheelbase);
    
    // Constructor to read parameters from a yaml file
    Config(const std::string& filename);
    
    // Default Constructor setting all values to a pre-tuned controller
    Config();
};

// Waypoints class containing x, y, theta
class Waypoints{
    private:
        // Function to read waypoints from a csv file
        void getWaypointsfromCSV(const string& filename);
        // Function to read waypoints from a ROS topic
        void getWaypointsfromPlanner(nav_msgs::msg::Path::ConstSharedPtr rrt_path_msg);
    public:
        // x coordinates of waypoints
        std::vector<float> x;
        // y coordinates of waypoints
        std::vector<float> y;
        // theta coordinates of waypoints
        std::vector<float> theta;
        // Function to get waypoints from a given config
        void getWaypoints(Config config, nav_msgs::msg::Path::ConstSharedPtr rrt_path_msg);
        // Default Constructor setting all values to empty vectors
        Waypoints();
};

float get_robot_yaw(geometry_msgs::msg::Pose pose);