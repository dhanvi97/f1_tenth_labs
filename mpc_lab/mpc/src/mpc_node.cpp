#include <mpc_model.h>
using std::placeholders::_1;
using namespace std;

class MPC : public rclcpp::Node
{
    // Implement MPC
    // This is just a template, you are free to implement your own node!

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_subscriber_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr rrt_subscriber_;
    Config config_;
    CarState car_state_;
    MPCSolver mpc_solver;
    Waypoints race_line;
    bool path_updated {false};
    float steer {0.0};
    float vel {1.5};
public:
    MPC() : Node("mpc_node")
    {
        // TODO: create ROS subscribers and publishers
        this->config_ = Config("/home/ros_ws/src/mpc/config/config.yaml");

        this->pose_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom", 1, std::bind(&MPC::pose_callback, this, _1));
        this->drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive", 1);
        this->path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(
            "/path", 1);
        this->plan_publisher_ = this->create_publisher<nav_msgs::msg::Path>(
            "/plan", 1);
        this->rrt_subscriber_ = this->create_subscription<nav_msgs::msg::Path>(
            this->config_.path_topic, 1, std::bind(&MPC::rrt_callback, this, _1));

        
        this->race_line.getWaypoints(this->config_, nullptr);
        this->mpc_solver = MPCSolver(config_);
    }

    void pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg)
    {
        // RCLCPP_INFO(this->get_logger(), "I heard: '%f'", odom_msg->pose.pose.position.x);
        float x = odom_msg->pose.pose.position.x;
        float y = odom_msg->pose.pose.position.y;
        float theta = get_robot_yaw(*odom_msg);
        this->car_state_ = CarState(x, y, theta, this->vel);
        tuple<double, double, vector<tuple<double ,double>>> control;
        if(this->race_line.x.empty()){
            RCLCPP_INFO(this->get_logger(), "Received empty path");
            this->vel = 0.0;
            // this->steer = 0.0;
        } else {
            this->mpc_solver.get_waypoints(this->race_line);
            control = this->mpc_solver.solve(this->car_state_, this->steer, this->vel);
            this->vel = get<1>(control);
            this->steer = get<0>(control);
        }
        ackermann_msgs::msg::AckermannDriveStamped drive_msg;
        drive_msg.header.stamp = this->now();
        drive_msg.header.frame_id = "map";
        drive_msg.drive.steering_angle = this->steer;
        drive_msg.drive.speed = this->vel;
        this->drive_publisher_->publish(drive_msg);
        path_publish();
        vector<tuple<double, double>> plan = get<2>(control);
        plan_publish(plan);

    }

    float get_robot_yaw(nav_msgs::msg::Odometry robot_odom){
        tf2::Quaternion q(
            robot_odom.pose.pose.orientation.x,
            robot_odom.pose.pose.orientation.y,
            robot_odom.pose.pose.orientation.z,
            robot_odom.pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw;
    }

    void path_publish(){
        
        nav_msgs::msg::Path path;
        path.header.frame_id = "map";
        path.header.stamp = this->now();
        std::vector<geometry_msgs::msg::PoseStamped> poses;
        for(int i = 0; i < int(this->race_line.x.size()); i++){
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.header.stamp = this->now();
            pose.pose.position.x = race_line.x.at(i);
            pose.pose.position.y = race_line.y.at(i);
            pose.pose.position.z = 0;
            pose.pose.orientation.x = 0;
            pose.pose.orientation.y = 0;
            pose.pose.orientation.z = 0;
            pose.pose.orientation.w = 1;
            poses.push_back(pose);
        }
        path.poses = poses;
        this->path_publisher_->publish(path);
    }

    void rrt_callback(const nav_msgs::msg::Path::ConstSharedPtr rrt_path_msg){
        this->race_line.getWaypoints(this->config_, rrt_path_msg);
    }

    void plan_publish(vector<tuple<double, double>> plan){
        nav_msgs::msg::Path path;
        path.header.frame_id = "map";
        path.header.stamp = this->now();
        vector<geometry_msgs::msg::PoseStamped> poses;
        for(int i = 0; i < int(plan.size()); i++){
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.header.stamp = this->now();
            pose.pose.position.x = get<0>(plan.at(i));
            pose.pose.position.y = get<1>(plan.at(i));
            pose.pose.position.z = 0;
            pose.pose.orientation.x = 0;
            pose.pose.orientation.y = 0;
            pose.pose.orientation.z = 0;
            pose.pose.orientation.w = 1;
            poses.push_back(pose);
        }
        path.poses = poses;
        this->plan_publisher_->publish(path);
    }
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MPC>());
    rclcpp::shutdown();
    return 0;
}