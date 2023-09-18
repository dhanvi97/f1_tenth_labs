#include "rclcpp/rclcpp.hpp"
/// CHECK: include needed ROS msg type headers and libraries
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <vector>
using std::placeholders::_1;
float FLOAT_MAX = std::numeric_limits<float>::infinity();
float TTC_THRESH = 1.25;


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
    float slip_angle = 0.0;
    /// TODO: create ROS subscribers and publishers
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_subscription_;
    sensor_msgs::msg::LaserScan prev_scan_msg = sensor_msgs::msg::LaserScan();

    void drive_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        /// TODO: update current speed
        this->speed = msg->twist.twist.linear.x;
        if(msg->twist.twist.linear.y < 0.001){
            this->slip_angle = 0;
        } else {
            this->slip_angle = std::atan(this->speed / msg->twist.twist.linear.y);
        }
    }

    std::vector<float> calc_rate_of_range_change(std::vector<float> angles, float slip_angle){
        std::vector<float> output;
        int N = int(angles.size());
        for(int i=0; i < N; i++){
            float del_r = this->speed*std::cos(angles.at(i) + slip_angle);
            if(del_r > 0){
                output.push_back(del_r);
            } else {
                output.push_back(0);
            } 
        }
        return output;
    }

    std::vector<float> calc_ittc_array(std::vector<float> range_rate, std::vector<float> ranges){
        std::vector<float> output;
        int N = int(range_rate.size());
        for(int i=0; i < N; i++){
            if(range_rate.at(i) > 0 && !std::isinf(ranges.at(i)) && !std::isnan(ranges.at(i))){
                output.push_back(ranges.at(i)/(range_rate.at(i)));
            }
            else {
                output.push_back(FLOAT_MAX);
            }
        }
        return output;
    }

    std::tuple<int, float> get_min_ittc(std::vector<float> ittc_array){
        float min = FLOAT_MAX;
        int index = 0;
        int N = int(ittc_array.size());
        for(int i=0; i < N; i++){
            if(ittc_array.at(i) < min){
                min = ittc_array.at(i);
                index = i;
            }
        }
        return std::make_tuple(index, min);
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        /// TODO: calculate TTC
        std::vector<float> angles;
        float theta = scan_msg->angle_min;
        bool stop_flag = false;
        while(theta <= scan_msg->angle_max){
            angles.push_back(theta);
            theta += scan_msg->angle_increment;
        }
        std::vector<float> del_r = calc_rate_of_range_change(angles, this->slip_angle);
        std::vector<float> ittc_array = calc_ittc_array(del_r, scan_msg->ranges);
        std::tuple<int, float> min_ittc = get_min_ittc(ittc_array);
        /// TODO: publish drive/brake message
        if(std::isinf(std::get<1>(min_ittc))) { 
            stop_flag = false;
        } else if(std::get<1>(min_ittc) < TTC_THRESH) {
            stop_flag = true;
            RCLCPP_INFO(
                this->get_logger(), "Minimum iTTC is '%f' seconds along '%f' angle", std::get<1>(min_ittc), angles.at(std::get<0>(min_ittc)) 
            );
        }

        if(stop_flag){
            auto drive_ack_msg = ackermann_msgs::msg::AckermannDriveStamped();
            drive_ack_msg.header.stamp = this->now();
            drive_ack_msg.drive.speed = 0.0;
            this->ackermann_publisher_->publish(drive_ack_msg);
        }
    }

};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}