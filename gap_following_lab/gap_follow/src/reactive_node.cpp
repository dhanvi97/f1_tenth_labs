#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
using std::placeholders::_1;
const double PI = 3.14159;
/// CHECK: include needed ROS msg type headers and libraries

class ReactiveFollowGap : public rclcpp::Node {
// Implement Reactive Follow Gap on the car
// This is just a template, you are free to implement your own node!

public:
    ReactiveFollowGap() : Node("reactive_node")
    {
        /// TODO: create ROS subscribers and publishers
        this->declare_parameter("avg_window_size", 3);
        this->declare_parameter("high_dist_cutoff", 3.0);
        this->declare_parameter("inflation_radius", 0.25);
        this->declare_parameter("high_speed", 5.0);
        this->declare_parameter("mid_speed", 4.0);
        this->declare_parameter("low_speed", 1.5);
        this->declare_parameter("disparity_thresh", 0.5);
        this->get_parameter("avg_window_size", avg_window_size_);
        this->get_parameter("high_dist_cutoff", high_dist_cutoff_);
        this->get_parameter("inflation_radius", inflation_radius_);
        this->get_parameter("high_speed", high_speed_);
        this->get_parameter("mid_speed", mid_speed_);
        this->get_parameter("low_speed", low_speed_);
        this->get_parameter("disparity_thresh", disparity_thresh_);
        laserscan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 1, std::bind(&ReactiveFollowGap::lidar_callback, this, _1));
        ackermann_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 1);
    }

private:
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";
    std::vector<float> pre_processed_ranges_;
    int avg_window_size_;
    double high_dist_cutoff_;
    double inflation_radius_;
    double angle_increment_;
    int range_size_;
    double range_max_;
    double angle_min_;
    double disparity_thresh_;
    double high_speed_, mid_speed_, low_speed_;
    int start_index_, end_index_;
    int fov_start_index, fov_end_index;
    double best_steering_angle_;
    /// TODO: create ROS subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_subscription_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_publisher_;

    void preprocess_lidar(float* ranges)
    {   
        // Preprocess the LiDAR scan array. Expert implementation includes:
        // 1.Setting each value to the mean over some window
        // 2.Rejecting high values (eg. > 3m)
        this->pre_processed_ranges_.clear();
        for(int i=0; i < this->range_size_; i++){
            double range_at_i = *(ranges + i);
            if(std::isinf(range_at_i) || std::isnan(range_at_i)){
                this->pre_processed_ranges_.push_back(this->range_max_);
            } else {
                this->pre_processed_ranges_.push_back(range_at_i);
            }
        }

        return;
    }

    void window_avg(){
        std::vector<float> averaged_ranges;
        for(int i =0; i < this->avg_window_size_; i++){
            averaged_ranges.push_back(this->pre_processed_ranges_.at(i));
        }
        for(int i=this->avg_window_size_; i < this->range_size_ - this->avg_window_size_; i++){
            float avg_range = 0.0;
            for(int j = i-this->avg_window_size_ + 1; j < i + this->avg_window_size_; j++){
                avg_range += this->pre_processed_ranges_.at(j);
            }
            avg_range /= 2*this->avg_window_size_ - 1;
            averaged_ranges.push_back(avg_range);
            
        }
        for(int i = this->range_size_ - this->avg_window_size_; i < this->range_size_; i++){
            averaged_ranges.push_back(this->pre_processed_ranges_.at(i));
        }
        this->pre_processed_ranges_ = averaged_ranges;
    }

    void distance_threshold(){
        for(int i=0; i < this->range_size_; i++){
            if(this->pre_processed_ranges_.at(i) > this->high_dist_cutoff_){
                this->pre_processed_ranges_.at(i) = this->high_dist_cutoff_;
            }
        }
    }

    void mark_safe_region(int min_index){
        double dist = this->pre_processed_ranges_.at(min_index);
        int num_of_cells = (this->inflation_radius_)/(dist*this->angle_increment_);
        for(int j = -num_of_cells; j < num_of_cells + 1; j++){
            int index_to_check = std::max(0, min_index + j);
            this->pre_processed_ranges_.at(std::min(index_to_check, this->range_size_ -1)) = 0.0;
        }

        return;
    }

    void find_max_gap()
    {   
        // Return the start index & end index of the max gap in free_space_ranges
        std::vector<int> zero_indices;
        zero_indices.push_back(fov_start_index);
        for(int i=this->fov_start_index; i <= this->fov_end_index; i++){
            if(this->pre_processed_ranges_.at(i) < 1.5){
                zero_indices.push_back(i);
            }
        }
        zero_indices.push_back(this->fov_end_index);
        int max_gap = 0;
        for(int i=0; i<int(zero_indices.size())-1; i++){
            int gap = zero_indices.at(i+1) - zero_indices.at(i);
            if(gap > max_gap){
                this->start_index_ = zero_indices.at(i);
                this->end_index_ = zero_indices.at(i+1);
                max_gap = gap;
            }
        }
        // RCLCPP_INFO(this->get_logger(), "Max gap size %d, with start and end indices %d, %d", max_gap, this->start_index_, this->end_index_);
        return;
    }

    void find_best_point()
    {   
        // Start_i & end_i are start and end indicies of max-gap range, respectively
        // Return index of best point in ranges
	    // Naive: Choose the furthest point within ranges and go there
        int best_point_ = 0;
        // double diff = std::max(std::abs(this->start_index_ - this->range_size_/2), std::abs(this->end_index_ - this->range_size_/2));
        double max_dist = -1.0;
        for(int i=this->start_index_; i < this->end_index_; i++ ){
            if(this->pre_processed_ranges_.at(i) > max_dist){
                max_dist = this->pre_processed_ranges_.at(i);
                best_point_ = i;
            }
        }
        // for(int i=this->start_index_; i < this->end_index_; i++ ){
        //     if(this->pre_processed_ranges_.at(i) == max_dist){
        //         if(std::abs(i - this->range_size_/2) < diff){
        //             best_point_ = i;
        //             diff = std::abs(i - this->range_size_/2);
        //         }
        //     }
        // }

        best_point_ = (this->start_index_ + this->end_index_)/2;
        this->best_steering_angle_ = this->angle_min_ + this->angle_increment_*best_point_;

        return;
    }

    int corner_checker(){
        for(int i=0; i < this->fov_start_index; i++){
            if(this->pre_processed_ranges_.at(i) < 0.1){
                return -1;
            }
        }
        for(int i=this->fov_end_index; i < this->range_size_; i++){
            if(this->pre_processed_ranges_.at(i) < 0.1){
                return 1;
            }
        }
        return 0;

    }

    bool steering_override_at_corner(int corner_coll){
        if(corner_coll == 0){
            return false;
        }
        else {
            if((corner_coll*this->best_steering_angle_) <= 0){
                return false;
            }
            else{
                return true;
            }
        }
    }

    void disparity_extender(){
        std::vector<float> disparity_ranges = this->pre_processed_ranges_;
        for(int i=this->fov_start_index; i < this->fov_end_index; i++){
            double disparity = this->pre_processed_ranges_.at(i+1) - this->pre_processed_ranges_.at(i);
            if(disparity > this->disparity_thresh_){
                double dist = this->pre_processed_ranges_.at(i);
                int num_of_cells = (0.45*this->inflation_radius_)/(dist*this->angle_increment_);
                for(int j=0; j < num_of_cells; j++){
                    int index_to_check = std::min(i+j, this->range_size_);
                    if(this->pre_processed_ranges_.at(index_to_check) > dist){
                        disparity_ranges.at(index_to_check) = dist;
                    }
                }
            }
            else if(disparity + this->disparity_thresh_ < 0) {
                double dist = this->pre_processed_ranges_.at(i+1);
                int num_of_cells = (0.45*this->inflation_radius_)/(dist*this->angle_increment_);
                for(int j=0; j < num_of_cells; j++){
                    int index_to_check = std::max(i-j, 0);
                    if(this->pre_processed_ranges_.at(index_to_check) > dist){
                        disparity_ranges.at(index_to_check) = dist;
                    }
                }
            }
        }
        this->pre_processed_ranges_ = disparity_ranges;
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {   
        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message

        /// TODO:
        this->range_size_ = int(scan_msg->ranges.size());
        this->angle_increment_ = scan_msg->angle_increment;
        this->angle_min_ = scan_msg->angle_min;
        this->range_max_ = scan_msg->range_max;
        this->fov_start_index = ((-PI/2) - this->angle_min_)/this->angle_increment_;
        this->fov_end_index = ((PI/2) - this->angle_min_)/this->angle_increment_;
        std::vector<float> range_data = scan_msg->ranges;
        preprocess_lidar(&range_data[0]);
        
        disparity_extender();
        int corner_collision = corner_checker();
        // window_avg();
        // distance_threshold();
        // Find closest point to LiDAR
        double min_dist = 100000;
        int min_index;
        for(int i=this->fov_start_index; i < this->fov_end_index; i++){
            if(this->pre_processed_ranges_.at(i) < min_dist){
                min_dist = this->pre_processed_ranges_.at(i);
                min_index = i;
            }
        }
        // RCLCPP_INFO(this->get_logger(), "Min indices in LIDAR array is %d", int(min_indices.size()));
        // Eliminate all points inside 'bubble' (set them to zero) 
        mark_safe_region(min_index);

        // Find max length gap
        find_max_gap();

        // Find the best point in the gap
        find_best_point(); 

        

        // Publish Drive message
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        double velocity = 0;
        if(std::abs(this->best_steering_angle_) > 20*(PI)/180.0){
            velocity = this->low_speed_;
        } else if(std::abs(this->best_steering_angle_) > 10*(PI)/180.0) {
            velocity = this->mid_speed_;
        } else {
            velocity = this->high_speed_;
        }
        drive_msg.drive.speed = velocity;
        if(!steering_override_at_corner(corner_collision)){
            drive_msg.drive.steering_angle = this->best_steering_angle_;
        }
        else {
            drive_msg.drive.steering_angle = 0.0;
            drive_msg.drive.speed = 1.0;
        }
        // RCLCPP_INFO(this->get_logger(), "Velocity, Steering: %f, %f", velocity, this->best_steering_angle_);
        this->ackermann_publisher_->publish(drive_msg);
    }



};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}