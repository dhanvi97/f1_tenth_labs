#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class Talker : public rclcpp::Node {
    public:
    Talker() : Node("talker") {
        v = 0.0;
        d = 0.0;
        this->declare_parameter("v", v);
        this->declare_parameter("d", d);
        this->get_parameter("v", v);
        this->get_parameter("d", d);

        publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 1);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1), std::bind(&Talker::publishMessage, this)
        );
    }
    private:
    void publishMessage(){
        auto message = ackermann_msgs::msg::AckermannDriveStamped();
        message.header.stamp = this->now();
        message.drive.steering_angle = this->d;
        message.drive.speed = this->v;
        this->publisher_->publish(message);
    }
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double v;
    double d;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Talker>());
    rclcpp::shutdown();
    return 0;

}