// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
// Make sure you have read through the header file as well

#include "rrt/rrt.h"

// Destructor of the RRT class
RRT::~RRT() {
    // Do something in here, free up used memory, print message, etc.
    RCLCPP_INFO(rclcpp::get_logger("RRT"), "%s\n", "RRT shutting down");
}

// Constructor of the RRT class
RRT::RRT(): rclcpp::Node("rrt_node"), gen((std::random_device())()) {

    // ROS publishers
    // TODO: create publishers for the the drive topic, and other topics you might need

    // ROS subscribers
    // TODO: create subscribers as you need
    string pose_topic = "ego_racecar/odom";
    string scan_topic = "/scan";
    string path_topic = "/rrt_path";
    string waypoints_topic = "/viz_path";
    string occupancy_grid_topic = "/occupancy_grid";
    
    pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      pose_topic, 1, std::bind(&RRT::pose_callback, this, std::placeholders::_1));
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, 1, std::bind(&RRT::scan_callback, this, std::placeholders::_1));
    rrt_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_topic, 1);
    viz_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/viz_path", 1);
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 1, std::bind(&RRT::map_callback, this, std::placeholders::_1));
    occupancy_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(occupancy_grid_topic, 1);

    // TODO: create a occupancy grid
    // In the car frame - lookahead_distance x lookahead_distance and resolution of occupancy_resolution
    // The car frame is centered at the rear axle, x is forward, y is left
    // center of grid hieght and left most column represents car position
    // make the grid all zeros
    readCSV("/home/ros_ws/src/lab6_pkg/data/atrium_waypoints.csv");
    this->lookahead_distance = 2.0;
    this->occupancy_resolution = 0.05;
    this->max_expansion_dist = 1.0;
    this->max_track_width = 1.0;
    this->occupancy_grid = vector<vector<int>>(int(lookahead_distance/occupancy_resolution), vector<int>(int(lookahead_distance/occupancy_resolution), 0));
    
    

    RCLCPP_INFO(rclcpp::get_logger("RRT"), "%s\n", "Created new RRT Object.");
}

void RRT::map_callback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_msg) {
    // The map callback, update your occupancy grid here
    // Args:
    //    map_msg (*OccupancyGrid): pointer to the incoming map message
    // Returns:
    //
    this->map = *map_msg;

}

void RRT::scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
    // The scan callback, update your occupancy grid here
    // Args:
    //    scan_msg (*LaserScan): pointer to the incoming scan message
    // Returns:
    //

    // TODO: update your occupancy grid
    this->updated_occupancy_grid = false;
    this->dynamic_obstacle = 0;

    float yaw = get_robot_yaw();

    for(int i = 0; i < scan_msg->ranges.size(); i++) {
        // check if scan comes from static map or dynamic obstacle
        if(scan_msg->ranges[i] < 0.2 || scan_msg->ranges[i] > 10.0) { continue; }
        float angle = scan_msg->angle_min + i*scan_msg->angle_increment;
        float range = scan_msg->ranges[i];
        if(angle < -M_PI/2 || angle > M_PI/2){
            continue;
        }
        // calculate coordinate of lidar point in map frame
        float point_x = range*cos(angle);
        float point_y = range*sin(angle);
        // convert point to map frame using robot yaw
        float map_x = point_x*cos(yaw) - point_y*sin(yaw) + this->robot_pose_.position.x;
        float map_y = point_x*sin(yaw) + point_y*cos(yaw) + this->robot_pose_.position.y;
        // convert to grid coordinates of this->map
        int grid_x = int((map_x - this->map.info.origin.position.x)/this->map.info.resolution);
        int grid_y = int((map_y - this->map.info.origin.position.y)/this->map.info.resolution);
        // check if in bounds
        if(grid_x < 0 || grid_x >= this->map.info.width || grid_y < 0 || grid_y >= this->map.info.height) {
            continue;
        }
        // check if point is occupied
        if(this->map.data[grid_y*this->map.info.width + grid_x] == 0){
            this->dynamic_obstacle++;
        }
    }
    RCLCPP_INFO(rclcpp::get_logger("RRT"), "Dynamic obstacle: %d", this->dynamic_obstacle);
    if(this->dynamic_obstacle > 0){
        this->lookahead_distance = 2.5;
        this->max_expansion_dist = 0.5;
        this->occupancy_grid = vector<vector<int>>(int(lookahead_distance/occupancy_resolution), vector<int>(int(lookahead_distance/occupancy_resolution), 0));
    } else {
        this->lookahead_distance = 2.5;
        this->max_expansion_dist = 0.5;
        this->occupancy_grid = vector<vector<int>>(int(lookahead_distance/occupancy_resolution), vector<int>(int(lookahead_distance/occupancy_resolution), 0));
    }

    create_local_occupancy_grid(scan_msg);
     
    // int N = int(occupancy_grid.size());
    // this->occupancy_grid.at(N/2).at(N/2) = 0;

                            
    

    nav_msgs::msg::OccupancyGrid occupancy_grid_msg;
    occupancy_grid_msg.header.frame_id = "map";
    occupancy_grid_msg.header.stamp = this->now();
    occupancy_grid_msg.info.resolution = this->occupancy_resolution;
    occupancy_grid_msg.info.width = this->occupancy_grid.size();
    occupancy_grid_msg.info.height = this->occupancy_grid.size();
    occupancy_grid_msg.info.origin = this->robot_pose_;
    occupancy_grid_msg.info.origin.position.x -= (this->lookahead_distance/2)*sqrt(2)*cos(yaw + M_PI/4);
    occupancy_grid_msg.info.origin.position.y -= (this->lookahead_distance/2)*sqrt(2)*sin(yaw + M_PI/4);

    vector<int8_t> data;
    for(int i=0; i < this->occupancy_grid.size(); i++){
        for(int j=0; j < this->occupancy_grid.at(i).size(); j++){
            if(this->occupancy_grid[i][j] == 1){
                data.push_back((int8_t) 100);
            } else {
                data.push_back((int8_t) 0);
            }
        }
    }
    occupancy_grid_msg.data = data;
    this->occupancy_pub_->publish(occupancy_grid_msg);
    this->updated_occupancy_grid = true;
    
}

void RRT::create_local_occupancy_grid(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg){
    // Clear occupancy grid
    for (auto &v : this->occupancy_grid)
        std::fill(v.begin(), v.end(), 0);
    
    float yaw = get_robot_yaw();

    for(int i = 0; i < scan_msg->ranges.size(); i++) {
        float angle = scan_msg->angle_min + i*scan_msg->angle_increment;
        float range = scan_msg->ranges[i];
        if(angle < -M_PI/2 || angle > M_PI/2){
            continue;
        }

        float point_x = range*cos(angle);
        float point_y = range*sin(angle);

        float map_x = point_x*cos(yaw) - point_y*sin(yaw) + this->robot_pose_.position.x;
        float map_y = point_x*sin(yaw) + point_y*cos(yaw) + this->robot_pose_.position.y;

        // convert to grid coordinates of occupancy grid
        int grid_x = int((point_x + this->lookahead_distance/2)/this->occupancy_resolution);
        int grid_y = int((point_y + this->lookahead_distance/2)/this->occupancy_resolution);

        // check if in bounds
        if(grid_x < 0 || grid_x >= this->occupancy_grid.at(0).size() || grid_y < 0 || grid_y >= this->occupancy_grid.size()) {
            continue;
        }        

        int inflation = int(0.2/this->occupancy_resolution);
        for(int a = -inflation; a <= inflation; a++) {
            for(int b = -inflation; b <= inflation; b++) {
                if(grid_y+a < 0 || grid_y+a >= this->occupancy_grid.size() || grid_x+b < 0 || grid_x+b >= this->occupancy_grid.size()){
                    continue;
                }
                this->occupancy_grid[grid_y+a][grid_x+b] = 1;
            }
        }
    } 
    


}

tuple<float , float> RRT::find_nearest_free_space(geometry_msgs::msg::Pose transformed_pose) {
    // Function to find nearest free space to a given point in the occupancy grid
    // Args:
    //    occupancy_grid (vector<vector<int>>): the occupancy grid
    //    x (int): x coordinate of the point
    //    y (int): y coordinate of the point
    // Returns:
    //    nearest_free_space (tuple<int, int>): the nearest free space to the point

    tuple<float, float> nearest_free_space;
    float min_dist = 1e8;
    int x = int((transformed_pose.position.x + this->lookahead_distance/2)/this->occupancy_resolution);
    int y = int((transformed_pose.position.y + this->lookahead_distance/2)/this->occupancy_resolution);

    x = int(max(x, 0));
    x = int(min(x, int(this->occupancy_grid.size()-1)));
    y = int(max(y, 0));
    y = int(min(y, int(this->occupancy_grid.size()-1)));

    if(this->occupancy_grid.at(y).at(x) == 0){
        // RCLCPP_INFO(rclcpp::get_logger("RRT"), "Goal in free space");
        return make_tuple(transformed_pose.position.x, transformed_pose.position.y);
    }
    
    // RCLCPP_INFO(rclcpp::get_logger("RRT"), "Shifting goal");

    for(int i= 0; i < this->occupancy_grid.size(); i++){
        for(int j=0; j < int(this->occupancy_grid.at(i).size()); j++){
            if(this->occupancy_grid.at(i).at(j) == 1) { continue; }
            float x_p = (j - int(this->occupancy_grid.size())/2)*this->occupancy_resolution;
            float y_p = (i - int(this->occupancy_grid.size())/2)*this->occupancy_resolution;
            float dist = 2*sqrt(pow(x_p - float(x), 2) + pow(y_p - float(y), 2)) - 0.01*x_p + 0.5*fabs(y_p);
            if(dist < min_dist){
                min_dist = dist;
                nearest_free_space = make_tuple(x_p, y_p);
            }
        }
    }
    // RCLCPP_INFO(rclcpp::get_logger("RRT"), "Shifting goal to %f, %f", get<0>(nearest_free_space),get<1>(nearest_free_space));
    return nearest_free_space;
}

void RRT::find_next_goal(int closest_waypoint_index) {
    // Find a goal point within the lookahead window which is 
    // closest to the waypoints in the csv file and as far as possible from the car

    //(x,y) is current car location , (x', y') is a given waypoint in the csv file
    float x_prime = get<0>(this->waypoints.at(closest_waypoint_index));
    float y_prime = get<1>(this->waypoints.at(closest_waypoint_index));
    geometry_msgs::msg::Pose goal_pose = geometry_msgs::msg::Pose();
    goal_pose.position.x = x_prime;
    goal_pose.position.y = y_prime;
    goal_pose.position.z = 0;
    goal_pose.orientation.x = 0;
    goal_pose.orientation.y = 0;
    goal_pose.orientation.z = 0;
    goal_pose.orientation.w = 1;

    geometry_msgs::msg::Pose transformed_pose = transform_to_vehicle_frame(goal_pose);
    float min_dist = sqrt(pow(transformed_pose.position.x, 2) + pow(transformed_pose.position.y, 2));
    tuple<float, float> col_row = find_nearest_free_space(transformed_pose);
    this->goal_x = get<0>(col_row);
    this->goal_y = get<1>(col_row);
    return;
}

geometry_msgs::msg::Pose RRT::transform_to_vehicle_frame(geometry_msgs::msg::Pose goal_pose){
        float x = goal_pose.position.x - this->robot_pose_.position.x;
        float y = goal_pose.position.y - this->robot_pose_.position.y;
        float yaw = get_robot_yaw();
        float theta = atan2(y, x);
        float del_theta = theta - yaw;
        geometry_msgs::msg::Pose transformed_pose;
        transformed_pose.position.x = sqrt(pow(x,2) + pow(y,2))*cos(del_theta);
        transformed_pose.position.y = sqrt(pow(x,2) + pow(y,2))*sin(del_theta);
        transformed_pose.orientation = goal_pose.orientation;
        return transformed_pose;
    }

int RRT::closest_point_on_waypoints(){
    // Returns index into waypoints for point at lookahead distance from a given robot pose
    float min_dist = 1000000;
    float best_index = -1;
    int min_index = -1;
    for(int i=0; i < int(this->waypoints.size()); i++){
        float dist = sqrt(pow(get<0>(this->waypoints[i]) - this->robot_pose_.position.x, 2) + pow(get<1>(this->waypoints[i]) - this->robot_pose_.position.y, 2));
        if(dist <= this->lookahead_distance/2){
            best_index = i;
        }
        if(dist < min_dist){
            min_dist = dist;
            min_index = i;
        }
    }
    if (best_index == -1){
        best_index = min_index;
    }

    // Make sure that point sampled is ahead of the car
    float x_prime = get<0>(this->waypoints.at(best_index));
    float y_prime = get<1>(this->waypoints.at(best_index));
    geometry_msgs::msg::Pose goal_pose = geometry_msgs::msg::Pose();
    goal_pose.position.x = x_prime;
    goal_pose.position.y = y_prime;
    goal_pose.position.z = 0;
    goal_pose.orientation.x = 0;
    goal_pose.orientation.y = 0;
    goal_pose.orientation.z = 0;
    goal_pose.orientation.w = 1;

    geometry_msgs::msg::Pose transformed_pose = transform_to_vehicle_frame(goal_pose);
    if(transformed_pose.position.x < 0.1){
        best_index = (min_index + int(this->lookahead_distance*50))%int(this->waypoints.size());
    }

    return best_index;

}

void RRT::pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg) {
    // The pose callback when subscribed to particle filter's inferred pose
    // The RRT main loop happens here
    // Args:
    //    pose_msg (*PoseStamped): pointer to the incoming pose message
    // Returns:
    //
    
    this->robot_pose_ = pose_msg->pose.pose;
    // tree as std::vector
    std::vector<RRT_Node> tree;
    std::vector<RRT_Node> path;
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id="map";
    path_msg.header.stamp = this->now();

    // TODO: fill in the RRT main loop
    RRT_Node root_node;
    // if(!this->updated_occupancy_grid) {
    //     this->rrt_path_pub_->publish(path_msg);
    //     return;
    // }
    root_node.x = 0.0;
    root_node.y = 0.0;
    root_node.cost = 0.0;
    root_node.is_root = true;
    if(check_collision(root_node, root_node)) {
        this->rrt_path_pub_->publish(path_msg);
        return;
    }

    find_next_goal(closest_point_on_waypoints());
    
    tree.push_back(root_node);
    
    int iter = 0;
    // RCLCPP_INFO(rclcpp::get_logger("RRT"), "Goal: %f, %f", this->goal_x, this->goal_y);
    // Add timeout 
    while(!is_goal(tree.back(), this->goal_x, this->goal_y) && iter < 2500){
        std::vector<double> sampled_point = sample(iter);
        int nearest_node_id = nearest(tree, sampled_point);
        RRT_Node nearest_node = tree.at(nearest_node_id);
        RRT_Node new_node = steer(nearest_node, sampled_point);
        if(check_collision(nearest_node, new_node)) 
        { 
            iter++;
            continue;
            
        }
        std::vector<int> neighbors = near(tree, new_node);
        int best_parent = nearest_node_id;
        if(neighbors.empty()){
            best_parent = nearest_node_id;
        } else {
            float min_cost = nearest_node.cost + line_cost(new_node, nearest_node);
            for(auto n : neighbors){
                if(check_collision(tree[n], new_node)) { continue; }
                float cost = tree[n].cost + line_cost(tree[n], new_node);
                if(cost < min_cost){
                    min_cost = cost;
                    best_parent = n;
                }
            }
        }
        new_node.parent = best_parent;
        new_node.cost = tree[best_parent].cost + line_cost(new_node, tree[best_parent]);
        tree.push_back(new_node);
        for(auto n : neighbors){
            if(check_collision(tree[n], new_node)) { continue; }
            float cost = new_node.cost + line_cost(tree[n], new_node);
            if(cost < tree[n].cost){
                tree[n].parent = tree.size() - 1;
                tree[n].cost = cost;
            }
        }
        iter++;
        
    }
    // RCLCPP_INFO(rclcpp::get_logger("RRT"), "Path found");
    if(iter < 2500){
        // RCLCPP_INFO(rclcpp::get_logger("RRT"), "Path found in %d iterations", iter);
        path = find_path(tree, tree.back());
    } else {
        std::vector<double> goal_point = {this->goal_x, this->goal_y};
        int alternate_path_end = nearest(tree, goal_point);
        path = find_path(tree, tree.at(alternate_path_end));
        RCLCPP_INFO(rclcpp::get_logger("RRT"), "Using alternate path");
    }
    
    
    // path found as Path message
    path_msg = upsample_path(path);
    this->rrt_path_pub_->publish(path_msg);
    visualize_waypoints();

}

nav_msgs::msg::Path RRT::upsample_path(std::vector<RRT_Node> &path){
    // Function to linearly interpolate between paths and create a navigation path
    // Args:
    //    path (std::vector<RRT_Node>): the path found by the RRT
    // Returns:
    //    upsampled_path (nav_msgs::msg::Path): the upsampled path
    float yaw = get_robot_yaw();
    nav_msgs::msg::Path upsampled_path;
    upsampled_path.header.frame_id = "map";
    upsampled_path.header.stamp = this->now();
    if(path.empty()){
        return upsampled_path;
    }
    
    for(int i=0; i < int(path.size()-1); i++){
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = this->now();
        float theta = atan2(path[i].y, path[i].x);
        pose.pose.position.x = sqrt(pow(path[i].x,2) + pow(path[i].y,2))*cos(theta + yaw) + this->robot_pose_.position.x;
        pose.pose.position.y = sqrt(pow(path[i].x,2) + pow(path[i].y,2))*sin(theta + yaw) + this->robot_pose_.position.y;
        pose.pose.position.z = 0;
        // convert yaw to quaternion
        float x = path[i].x;
        float y = path[i].y;
        float x_prime = path[i+1].x;
        float y_prime = path[i+1].y;
        float del_x = x_prime - x;
        float del_y = y_prime - y;
        float path_orientation = atan2(del_y, del_x);

        geometry_msgs::msg::Quaternion q = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), path_orientation + yaw));
        pose.pose.orientation = q;
        upsampled_path.poses.push_back(pose);
        float lambda = 0.1;
        while(lambda < 1){
            geometry_msgs::msg::PoseStamped pose_subsample;
            pose_subsample.header.frame_id = "map";
            pose_subsample.header.stamp = this->now();
            float theta_p = atan2(y + lambda*del_y, x + lambda*del_x);
            pose_subsample.pose.position.x = sqrt(pow(x + lambda*del_x,2) + pow(y + lambda*del_y,2))*cos(theta_p + yaw) + this->robot_pose_.position.x;
            pose_subsample.pose.position.y = sqrt(pow(x + lambda*del_x,2) + pow(y + lambda*del_y,2))*sin(theta_p + yaw) + this->robot_pose_.position.y;
            pose_subsample.pose.position.z = 0;
            pose_subsample.pose.orientation = pose.pose.orientation;
            upsampled_path.poses.push_back(pose_subsample);
            lambda += 0.1;
        }
    }
    return upsampled_path;
}

float RRT::get_robot_yaw(){
    tf2::Quaternion q(this->robot_pose_.orientation.x, 
                      this->robot_pose_.orientation.y, 
                      this->robot_pose_.orientation.z, 
                      this->robot_pose_.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

std::vector<double> RRT::sample(int iteration) {
    // This method returns a sampled point from the free space
    // You should restrict so that it only samples a small region
    // of interest around the car's current position
    // Args:
    // Returns:
    //     sampled_point (std::vector<double>): the sampled point in free space

    std::vector<double> sampled_point;

    
    // TODO: fill in this method
    // look up the documentation on how to use std::mt19937 devices with a distribution
    // the generator and the distribution is created for you (check the header file)
    // you can use the generator like this:
    tuple<float, float> x_lim, y_lim;
    if(iteration % 50 == 0){
        x_lim = make_tuple(max(this->goal_x - float(0.1), float(-this->lookahead_distance/2)), min(this->goal_x + float(0.1), float(this->lookahead_distance/2)));
        y_lim = make_tuple(max(this->goal_y - float(0.1), float(-this->lookahead_distance/2)), min(this->goal_y + float(0.1), float(this->lookahead_distance/2)));
    } else {
        x_lim = make_tuple(this->occupancy_resolution, float(this->lookahead_distance/2));
        y_lim = make_tuple(float(-this->lookahead_distance/2), float(this->lookahead_distance/2));
    }

    std::uniform_real_distribution<> x_dist(get<0>(x_lim), get<1>(x_lim));
    float sampled_x = x_dist(this->gen);

    std::uniform_real_distribution<> y_dist(get<0>(y_lim), get<1>(y_lim));
    float sampled_y = y_dist(this->gen);

    int x_samp = int((sampled_x + this->lookahead_distance/2)/this->occupancy_resolution);
    int y_samp = int((sampled_y + this->lookahead_distance/2)/this->occupancy_resolution);
    x_samp = int(max(x_samp, 0));
    x_samp = int(min(x_samp, int(this->occupancy_grid.size()-1)));
    y_samp = int(max(y_samp, 0));
    y_samp = int(min(y_samp, int(this->occupancy_grid.size()-1)));


    while(this->occupancy_grid[y_samp][x_samp] == 1) {
        sampled_x = x_dist(this->gen);
        sampled_y = y_dist(this->gen);
        x_samp = int((sampled_x + this->lookahead_distance/2)/this->occupancy_resolution);
        y_samp = int((sampled_y + this->lookahead_distance/2)/this->occupancy_resolution);
    }

    sampled_point.push_back(sampled_x);
    sampled_point.push_back(sampled_y);
    
    return sampled_point;
}


int RRT::nearest(std::vector<RRT_Node> &tree, std::vector<double> &sampled_point) {
    // This method returns the nearest node on the tree to the sampled point
    // Args:
    //     tree (std::vector<RRT_Node>): the current RRT tree
    //     sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //     nearest_node (int): index of nearest node on the tree

    int nearest_node = 0;
    // TODO: fill in this method
    double min_dist = 1000000;
    for(int i=0; i < int(tree.size()); i++){
        float dist = sqrt(pow(tree[i].x - sampled_point[0], 2) + pow(tree[i].y - sampled_point[1], 2));
        if(dist < min_dist){
            min_dist = dist;
            nearest_node = i;
        }
    }
    return nearest_node;
}

RRT_Node RRT::steer(RRT_Node &nearest_node, std::vector<double> &sampled_point) {
    // The function steer:(x,y)->z returns a point such that z is “closer” 
    // to y than x is. The point z returned by the function steer will be 
    // such that z minimizes ||z−y|| while at the same time maintaining 
    //||z−x|| <= max_expansion_dist, for a prespecified max_expansion_dist > 0

    // basically, expand the tree towards the sample point (within a max dist)

    // Args:
    //    nearest_node (RRT_Node): nearest node on the tree to the sampled point
    //    sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //    new_node (RRT_Node): new node created from steering

    RRT_Node new_node;
    new_node.is_root = false;
    // TODO: fill in this method
    
    float exp_dist = this->max_expansion_dist;
    float total_dist = sqrt(pow(sampled_point[0] - nearest_node.x, 2) + pow(sampled_point[1] - nearest_node.y, 2));
    new_node.x = nearest_node.x + (sampled_point[0] - nearest_node.x)*(this->max_expansion_dist/total_dist);
    new_node.y = nearest_node.y + (sampled_point[1] - nearest_node.y)*(this->max_expansion_dist/total_dist);
    return new_node;
}

bool RRT::check_collision(RRT_Node &nearest_node, RRT_Node &new_node) {
    // This method returns a boolean indicating if the path between the 
    // nearest node and the new node created from steering is collision free
    // Args:
    //    nearest_node (RRT_Node): nearest node on the tree to the sampled point
    //    new_node (RRT_Node): new node created from steering
    // Returns:
    //    collision (bool): true if in collision, false otherwise

    bool collision = false;
    // TODO: fill in this method
    float lambda = 0;
    // float total_dist = sqrt(pow(new_node.x - nearest_node.x, 2) + pow(new_node.y - nearest_node.y, 2));
    while(lambda <= 1){
        float x = nearest_node.x + (lambda)*(new_node.x - nearest_node.x);
        float y = nearest_node.y + (lambda)*(new_node.y - nearest_node.y);
        int grid_x = int((x + this->lookahead_distance/2)/this->occupancy_resolution);
        int grid_y = int((y + this->lookahead_distance/2)/this->occupancy_resolution);
        grid_x = int(max(grid_x, 0));
        grid_x = int(min(grid_x, int(this->occupancy_grid.size()-1)));
        grid_y = int(max(grid_y, 0));
        grid_y = int(min(grid_y, int(this->occupancy_grid.size()-1)));

        if(this->occupancy_grid[grid_y][grid_x] == 1) {
            collision = true;
            break;
        }
        lambda += 0.1;
    }
    return collision;
}

bool RRT::is_goal(RRT_Node &latest_added_node, double goal_x, double goal_y) {
    // This method checks if the latest node added to the tree is close
    // enough (defined by goal_threshold) to the goal so we can terminate
    // the search and find a path
    // Args:
    //   latest_added_node (RRT_Node): latest addition to the tree
    //   goal_x (double): x coordinate of the current goal
    //   goal_y (double): y coordinate of the current goal
    // Returns:
    //   close_enough (bool): true if node close enough to the goal

    bool close_enough = false;
    // TODO: fill in this method
    float dist = sqrt(pow(latest_added_node.x - goal_x, 2) + pow(latest_added_node.y - goal_y, 2));
    if(dist < 0.1) {
        close_enough = true;
    }
    return close_enough;
}

std::vector<RRT_Node> RRT::find_path(std::vector<RRT_Node> &tree, RRT_Node &latest_added_node) {
    // This method traverses the tree from the node that has been determined
    // as goal
    // Args:
    //   latest_added_node (RRT_Node): latest addition to the tree that has been
    //      determined to be close enough to the goal
    // Returns:
    //   path (std::vector<RRT_Node>): the vector that represents the order of
    //      of the nodes traversed as the found path
    
    std::vector<RRT_Node> found_path;
    // TODO: fill in this method
    while(!latest_added_node.is_root) {
        found_path.push_back(latest_added_node);
        latest_added_node = tree[latest_added_node.parent];
    }
    // Adding root since while loop breaks before that
    found_path.push_back(latest_added_node);
    std::reverse(found_path.begin(), found_path.end());
    return found_path;
}

// RRT* methods
double RRT::cost(std::vector<RRT_Node> &tree, RRT_Node &node) {
    // This method returns the cost associated with a node
    // Args:
    //    tree (std::vector<RRT_Node>): the current tree
    //    node (RRT_Node): the node the cost is calculated for
    // Returns:
    //    cost (double): the cost value associated with the node

    double cost = 0;
    // TODO: fill in this method
    RRT_Node parent_node = tree[node.parent];
    cost = parent_node.cost + line_cost(parent_node, node);

    return cost;
}

double RRT::line_cost(RRT_Node &n1, RRT_Node &n2) {
    // This method returns the cost of the straight line path between two nodes
    // Args:
    //    n1 (RRT_Node): the RRT_Node at one end of the path
    //    n2 (RRT_Node): the RRT_Node at the other end of the path
    // Returns:
    //    cost (double): the cost value associated with the path

    double cost = 0;
    // TODO: fill in this method
    // Using L2 norm in absence of better metric
    cost = sqrt(pow(n1.x - n2.x, 2) + pow(n1.y - n2.y, 2));

    return cost;
}

std::vector<int> RRT::near(std::vector<RRT_Node> &tree, RRT_Node &node) {
    // This method returns the set of Nodes in the neighborhood of a 
    // node.
    // Args:
    //   tree (std::vector<RRT_Node>): the current tree
    //   node (RRT_Node): the node to find the neighborhood for
    // Returns:
    //   neighborhood (std::vector<int>): the index of the nodes in the neighborhood

    std::vector<int> neighborhood;
    // TODO:: fill in this method
    float neighborhood_radius = this->lookahead_distance/4.0;
    for(int i=0; i < int(tree.size()); i++){
        float dist = sqrt(pow(tree[i].x - node.x, 2) + pow(tree[i].y - node.y, 2));
        if(dist < neighborhood_radius){
            neighborhood.push_back(i);
        }
    }

    return neighborhood;
}

void RRT::readCSV(const std::string& filename) {
        std::vector<float> column1, column2, column3, column4;

        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Failed to open the file: " << filename << std::endl;
            for(int i=0; i < int(column1.size()); i++) {
                this->waypoints.push_back(std::make_tuple(column1[i], column2[i]));
            }
            return;
        }

        std::string line;
        while (std::getline(file, line)) {
            std::istringstream lineStream(line);
            std::string cell;
            std::vector<std::string> tokens;

            while (std::getline(lineStream, cell, ',')) {
                tokens.push_back(cell);
            }

            if (tokens.size() == 3) {
                column1.push_back(std::stof(tokens[0]));
                column2.push_back(std::stof(tokens[1]));
                column3.push_back(std::stof(tokens[2]));
                // column4.push_back(std::stof(tokens[3]));
            }
        }

        file.close();
        
        for(int i=0; i < int(column1.size()); i++) {
            this->waypoints.push_back(std::make_tuple(column1[i], column2[i]));
        }
    }

void RRT::visualize_waypoints(){
    // Publish nav_msgs Path with this->waypoints
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = this->now();
    for(int i=0; i < int(this->waypoints.size()); i++){
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = this->now();
        pose.pose.position.x = get<0>(this->waypoints[i]);
        pose.pose.position.y = get<1>(this->waypoints[i]);
        pose.pose.position.z = 0;
        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        pose.pose.orientation.w = 1;
        path_msg.poses.push_back(pose);
    }
    this->viz_path_pub_->publish(path_msg);
}
