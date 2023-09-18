#include "rclcpp/rclcpp.hpp"
#include <string>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
using std::placeholders::_1;
const double DIST_FROM_WALL = 1.2; // In meters
const double PI = 3.14159;

class WallFollow : public rclcpp::Node {

public:
    WallFollow() : Node("wall_follow_node")
    {
        // TODO: create ROS subscribers and publishers
        laserscan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 1, std::bind(&WallFollow::scan_callback, this, _1));
        ackermann_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 1);
    }

private:
    // PID CONTROL PARAMS
    double kp = 1.00;
    double kd = 0.005;
    double ki = 0.005;
    double servo_offset = 0.0;
    double prev_error = 0.0;
    double error = 0.0;
    double integral = 0.0;

    // Topics
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";
    /// TODO: create ROS subscribers and publishers
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_subscription_;
    // Helper members
    double angle_increment = 0.0;
    double angle_min = 0.0;
    double angle_max = 0.0;
    double range_max = 0.0;
    double range_min = 0.0;
    double theta_a = 0.0;
    double theta_b = 0.0;
    double current_steering_angle = 0.0;
    rclcpp::Time prev_time = this->now();
    double lookahead = 1.5;


    std::tuple<double, double> get_range(float* range_data, double angle)
    {
        /*
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle
            angle: If range was inf/nan at actual angle, we return the closest angle that has valid values
        */

        // TODO: implement
        int index_in_range_list = (angle - this->angle_min)/this->angle_increment ;
        float range = *(range_data + index_in_range_list);
        int i = 0;
        if(std::isinf(range) || std::isnan(range)){
            // Find the closest range that is valid and adjust theta accordingly
            i = 1;
            while(!(std::isinf(range) || std::isnan(range))){
                range = *(range_data + index_in_range_list + i);
                if(i > 0){
                    i *= -1;
                } else {
                    i = -1*i + 1;
                }
            }
        }
        double used_angle = this->angle_min + this->angle_increment*(index_in_range_list + i);

        return std::make_tuple(range, used_angle);
    }

    double get_error(float* range_data, double dist)
    {
        /*
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        */

        // TODO:implement
        double theta_a = (PI/4.0);
        double theta_b = (PI/2.0);
        std::tuple<double,double> r_a = get_range(range_data, theta_a);
        std::tuple<double,double> r_b = get_range(range_data, theta_b);
        double a = std::get<0>(r_a);
        theta_a = std::get<1>(r_a);
        double b = std::get<0>(r_b);
        theta_b = std::get<1>(r_b);
        double delta_theta = theta_b - theta_a;
        double alpha = std::atan2((a*std::cos(delta_theta) - b), (a*std::sin(delta_theta)));
        std::cout << "Distance to wall is: " << b*std::cos(alpha) << std::endl;
        double d_t = this->lookahead*std::sin(alpha) + b*std::cos(alpha);
        std::cout << "Lookahead correction: " << this->lookahead*std::sin(alpha) << std::endl;
        double error = dist - d_t;
        return error;
    }

    void pid_control(double error, double velocity)
    {
        /*
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        */
        // TODO: Use kp, ki & kd to implement a PID controller
        double delta_t = (this->now() - prev_time).nanoseconds()/std::pow(10,9);
        this->integral += error*delta_t;
        double v_theta = -(this->kp*error + this->kd*(error - this->prev_error)/delta_t + this->ki*this->integral);
        this->prev_time = this->now();
        this->prev_error = error;
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        // TODO: fill in drive message and publish
        drive_msg.drive.speed = velocity;
        drive_msg.drive.steering_angle = v_theta;
        this->ackermann_publisher_->publish(drive_msg);
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        /*
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        */
        std::vector<float> ranges = scan_msg->ranges;
        this->angle_increment = scan_msg->angle_increment;
        this->angle_min = scan_msg->angle_min;
        this->angle_max = scan_msg->angle_max;
        this->range_max = scan_msg->range_min;
        this->range_min = scan_msg->range_max;
        double error = get_error(&ranges[0], DIST_FROM_WALL); // TODO: replace with error calculated by get_error()
        double velocity = 0.0; // TODO: calculate desired car velocity based on error
        if(std::abs(this->current_steering_angle) > 20*(PI)/180.0){
            velocity = 0.5;
        } else if(std::abs(this->current_steering_angle) > 10*(PI)/180.0) {
            velocity = 1.0;
        } else {
            velocity = 1.5;
        }
        // TODO: actuate the car with PID
        pid_control(error, velocity);

    }

};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollow>());
    rclcpp::shutdown();
    return 0;
}