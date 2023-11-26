#include "util.h"

CarState::CarState(float x, float y, float theta, float v){
    this->x = x;
    this->y = y;
    this->theta = theta;
    this->v = v;
}

CarState::CarState(){
    this->x = 0;
    this->y = 0;
    this->theta = 0;
    this->v = 0;
}

Config::Config(float dt, float dtk, int N, float Q[3], float R[2], float QN[3], float v_max, float v_min, float delta_max, float delta_min, float wheelbase){
    this->dt = dt;
    this->dtk = dtk;
    this->N = N;
    this->v_max = v_max;
    this->v_min = v_min;
    this->delta_max = delta_max;
    this->delta_min = delta_min;
    for(int i = 0; i < 3; i++){
        this->Q[i] = Q[i];
        this->QN[i] = QN[i];
    }
    for(int i = 0; i < 2; i++){
        this->R[i] = R[i];
    }
    this->wheelbase = wheelbase;
}

Config::Config(){
    this->dt = 0.1;
    this->dtk = 0.01;
    this->N = 8;
    this->v_max = 1.5;
    this->v_min = 1.2;
    this->delta_max = 0.4;
    this->delta_min = -0.4;
    for(int i = 0; i < 3; i++){
        this->Q[i] = 1.0;
        this->QN[i] = 1.0;
    }
    for(int i = 0; i < 2; i++){
        this->R[i] = 0.01;
    }
    this->wheelbase = 0.33;
}

Waypoints::Waypoints() {
    this->x = std::vector<float>();
    this->y = std::vector<float>();
    this->theta = std::vector<float>();
}

void Waypoints::getWaypoints(Config config, nav_msgs::msg::Path::ConstSharedPtr path_ = nullptr){
    if(config.path_source == "csv"){
        this->getWaypointsfromCSV(config.path_filename);
    }
    else if(config.path_source == "planner"){
        this->getWaypointsfromPlanner(path_);
    }
    else{
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid path source");
    }
}

void Waypoints::getWaypointsfromCSV(const string& filename){
    std::vector<float> column1, column2, column3;

    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open the file: " << filename << std::endl;
        for(int i=0; i < int(column1.size()); i++) {
            this->x.push_back(column1[i]);
            this->y.push_back(column2[i]);
            this->theta.push_back(column3[i]);
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

        if (tokens.size() >= 3) {
            column1.push_back(std::stof(tokens[0]));
            column2.push_back(std::stof(tokens[1]));
            column3.push_back(std::stof(tokens[2]));
        }
    }

    file.close();
    
    for(int i=0; i < int(column1.size()); i++) {
        this->x.push_back(column1[i]);
        this->y.push_back(column2[i]);
        this->theta.push_back(column3[i]);
    }
}

void Waypoints::getWaypointsfromPlanner(nav_msgs::msg::Path::ConstSharedPtr rrt_path_msg){
    if(rrt_path_msg == nullptr){
        return;
    }
    this->x.clear();
    this->y.clear();
    this->theta.clear();
    if(rrt_path_msg->poses.empty()){
        return;
    }
    for(int i = 0; i < int(rrt_path_msg->poses.size()); i++){
        this->x.push_back(rrt_path_msg->poses[i].pose.position.x);
        this->y.push_back(rrt_path_msg->poses[i].pose.position.y);
        this->theta.push_back(get_robot_yaw(rrt_path_msg->poses[i].pose));
    }
}

Config::Config(const std::string& filename){
    try {
        // Load the YAML file
        YAML::Node config = YAML::LoadFile(filename);

        this->N = config["N"].as<int>();
        this->dt = config["dt"].as<float>();
        this->dtk = config["dtk"].as<float>();
        this->v_max = config["v_max"].as<float>();
        this->v_min = config["v_min"].as<float>();
        this->delta_max = config["delta_max"].as<float>();
        this->delta_min = config["delta_min"].as<float>();
        for(int i = 0; i < 3; i++){
            this->Q[i] = config["Q"][i].as<float>();
            this->QN[i] = config["QN"][i].as<float>();
        }
        for(int i = 0; i < 2; i++){
            this->R[i] = config["R"][i].as<float>();
        }
        this->wheelbase = config["wheelbase"].as<float>();
        this->operation_mode = config["operation_mode"].as<std::string>();
        this->path_source = config["path_source"].as<std::string>();
        this->path_filename = config["path_filename"].as<std::string>();
        this->path_topic = config["path_topic"].as<std::string>();

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Config loaded from %s", filename.c_str());
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "N: %d", this->N);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "dt: %f", this->dt);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "dtk: %f", this->dtk);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "v_max: %f", this->v_max);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "v_min: %f", this->v_min);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "delta_max: %f", this->delta_max);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "delta_min: %f", this->delta_min);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "wheelbase: %f", this->wheelbase);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "operation_mode: %s", this->operation_mode.c_str());
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "path_source: %s", this->path_source.c_str());
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "path_filename: %s", this->path_filename.c_str());
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "path_topic: %s", this->path_topic.c_str());

        for(int i = 0; i < 3; i++){
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Q[%d]: %f", i, this->Q[i]);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "QN[%d]: %f", i, this->QN[i]);
        }
        for(int i = 0; i < 2; i++){
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "R[%d]: %f", i, this->R[i]);
        }
        
    } catch (const YAML::Exception& e) {
        std::cerr << "Error reading YAML file: " << e.what() << std::endl;
    }
}

float get_robot_yaw(geometry_msgs::msg::Pose pose){
    tf2::Quaternion q(pose.orientation.x, 
                      pose.orientation.y, 
                      pose.orientation.z, 
                      pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}