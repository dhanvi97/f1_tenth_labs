#include "rclcpp/rclcpp.hpp"
/// CHECK: include needed ROS msg type headers and libraries
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <vector>
using std::placeholders::_1;
float FLOAT_MAX = std::numeric_limits<float>::infinity();


class Safety : public rclcpp::Node {
// The class that handles emergency braking

public:
    Safety() : Node("safety_node")
    {
        /*
        You should also subscribe to the /scan topic to get the
        sensor_msgs/LaserScan messages and the /ego_racecar/odom topic to get
        the nav_msgs/Odometry messages

        The subscribers should use the provided odom_callback and 
        scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        */

        /// TODO: create ROS subscribers and publishers
        laserscan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 1, std::bind(&Safety::scan_callback, this, _1));
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom", 1, std::bind(&Safety::drive_callback, this, _1));
        ackermann_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 1);
        
    }

private:
    double speed = 0.0;
    /// TODO: create ROS subscribers and publishers
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_subscription_;
    sensor_msgs::msg::LaserScan prev_scan_msg = sensor_msgs::msg::LaserScan();

    void drive_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        /// TODO: update current speed
        this->speed = msg->twist.twist.linear.x;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        /// TODO: calculate TTC
        double del_t_seconds = scan_msg->header.stamp.sec - this->prev_scan_msg.header.stamp.sec;
        std::vector<float> ttc_array(this->prev_scan_msg.ranges.size());
        for(int i=0; i < int(this->prev_scan_msg.ranges.size()); i++){
            double del_range = scan_msg->ranges.at(i) - this->prev_scan_msg.ranges.at(i);
            double range_change_rate = del_range/del_t_seconds;
            if(range_change_rate < 0){
                ttc_array[i] = scan_msg->ranges.at(i) / (-1*range_change_rate);
            }
            else {
                ttc_array[i] = FLOAT_MAX;
            }
        }
        /// TODO: publish drive/brake message
    }





};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}