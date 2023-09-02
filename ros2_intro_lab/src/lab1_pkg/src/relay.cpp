#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
using std::placeholders::_1;

class Relay : public rclcpp::Node{
    public:
    Relay() : Node("relay") {
        subscription_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "drive", 1, std::bind(&Relay::subCallback, this, _1));
        publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive_relay", 1);
    }
    private:
    void subCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg){
        float incoming_speed = msg->drive.speed;
        float incoming_steering_angle = msg->drive.steering_angle;

        float output_speed = 3*incoming_speed;
        float output_angle = 3*incoming_steering_angle;

        auto out_msg = ackermann_msgs::msg::AckermannDriveStamped();

        out_msg.header.stamp = this->now();
        out_msg.drive.speed = output_speed;
        out_msg.drive.steering_angle = output_angle;
        publisher_->publish(out_msg);
    }
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr subscription_;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Relay>());
    rclcpp::shutdown();
    return 0;
}