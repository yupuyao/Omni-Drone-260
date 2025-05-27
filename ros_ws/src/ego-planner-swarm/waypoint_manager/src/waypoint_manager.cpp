// waypoint_manager.cpp
// This node manages predefined waypoints and processes visual module inputs
// to control drone navigation and operation sequences

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "k230_msgs/msg/visual_data.hpp"
#include <vector>
#include <chrono>
#include <thread>
#include <atomic>
#include <cstdlib>

// Atomic boolean for landing trigger
std::atomic<bool> land_triggered_{false};

class WaypointManager : public rclcpp::Node {
public:
    WaypointManager() : Node("waypoint_manager") {
        // Initialize publisher for waypoints
        goal_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/move_base_simple/goal", 10);
        
        // Initialize subscriber for visual module data
        visual_subscriber_ = this->create_subscription<k230_msgs::msg::VisualData>(
            "/visual_module/data", 10, 
            std::bind(&WaypointManager::visualDataCallback, this, std::placeholders::_1));
        
        // Load waypoints from parameter file
        loadWaypoints();
        
        RCLCPP_INFO(this->get_logger(), "Waypoint Manager initialized");
    }

    // Start publishing the predefined waypoints sequence
    void startWaypointSequence() {
        RCLCPP_INFO(this->get_logger(), "Starting waypoint sequence");
        
        in_visual_control_mode_ = false;
        current_waypoint_index_ = 0;
        
        // Create a timer to publish waypoints with 100ms interval
        waypoint_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&WaypointManager::publishNextWaypoint, this));
    }

private:
    // Publisher for waypoints
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
    
    // Subscriber for visual module data
    rclcpp::Subscription<k230_msgs::msg::VisualData>::SharedPtr visual_subscriber_;
    
    // Timer for publishing waypoints in sequence
    rclcpp::TimerBase::SharedPtr waypoint_timer_;
    
    // Vector to store preset waypoints
    std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
    
    // Current index in the waypoint sequence
    size_t current_waypoint_index_ = 0;
    
    // Flag to indicate if we're in visual control mode
    bool in_visual_control_mode_ = false;
    
    // Load waypoints from parameter file
    void loadWaypoints() {
        // Declare and get parameters for waypoints
        this->declare_parameter("waypoints.count", 0);
        int count = this->get_parameter("waypoints.count").as_int();
        
        for (int i = 0; i < count; i++) {
            std::string prefix = "waypoints.point" + std::to_string(i) + ".";
            
            // Declare parameters for each waypoint
            this->declare_parameter(prefix + "x", 0.0);
            this->declare_parameter(prefix + "y", 0.0);
            this->declare_parameter(prefix + "z", 0.0);
            
            // Get parameter values
            double x = this->get_parameter(prefix + "x").as_double();
            double y = this->get_parameter(prefix + "y").as_double();
            double z = this->get_parameter(prefix + "z").as_double();
            
            // Create waypoint
            auto waypoint = geometry_msgs::msg::PoseStamped();
            waypoint.header.frame_id = "world";
            waypoint.pose.position.x = x;
            waypoint.pose.position.y = y;
            waypoint.pose.position.z = z;
            waypoints_.push_back(waypoint);
            
            RCLCPP_INFO(this->get_logger(), "Loaded waypoint %d: (%.2f, %.2f, %.2f)", i, x, y, z);
        }
        
        RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints from parameters", waypoints_.size());
    }
    
    // Publish the next waypoint in sequence
    void publishNextWaypoint() {
        if (current_waypoint_index_ >= waypoints_.size()) {
            // All waypoints have been published, switch to visual control mode
            waypoint_timer_->cancel();
            in_visual_control_mode_ = true;
            RCLCPP_INFO(this->get_logger(), "All preset waypoints published, switching to visual control mode");
            return;
        }
        
        // Get the current waypoint
        auto waypoint = waypoints_[current_waypoint_index_];
        waypoint.header.stamp = this->now();
        
        // Publish the waypoint
        goal_publisher_->publish(waypoint);
        
        RCLCPP_INFO(this->get_logger(), "Published waypoint %zu: (%.2f, %.2f, %.2f)",
                   current_waypoint_index_, 
                   waypoint.pose.position.x,
                   waypoint.pose.position.y,
                   waypoint.pose.position.z);
        
        // Move to the next waypoint
        current_waypoint_index_++;
    }
    
    // Publish a waypoint with the given coordinates
    void publishWaypoint(float x, float y, float z) {
        auto waypoint = geometry_msgs::msg::PoseStamped();
        waypoint.header.stamp = this->now();
        waypoint.header.frame_id = "world";
        waypoint.pose.position.x = x;
        waypoint.pose.position.y = y;
        waypoint.pose.position.z = z;
        
        goal_publisher_->publish(waypoint);
        
        RCLCPP_INFO(this->get_logger(), "Published waypoint: (%.2f, %.2f, %.2f)", x, y, z);
    }
    
    // Execute a script with sudo
    void executeScript(const std::string& script_name) {
        std::string command = "sudo /home/orangepi/utils/" + script_name;
        RCLCPP_INFO(this->get_logger(), "Executing script: %s", command.c_str());
        
        int result = std::system(command.c_str());
        if (result != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to execute script: %s", command.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "Script executed successfully");
        }
    }
    
    // Callback for visual module data
    void visualDataCallback(const k230_msgs::msg::VisualData::SharedPtr msg) {
        if (!in_visual_control_mode_) {
            return;  // Ignore visual data until preset waypoints are published
        }
        
        float x = msg->x;
        float y = msg->y;
        float z = msg->z;
        std::string flag = msg->flag;
        
        RCLCPP_INFO(this->get_logger(), "Received visual data: pos=(%.2f, %.2f, %.2f), flag=%s",
                   x, y, z, flag.c_str());
        
        // Always publish the waypoint first regardless of flag
        publishWaypoint(x, y, z);
        
        // Process based on flag
        if (flag == "clamp") {
            // Execute clamp script after publishing waypoint
            executeScript("clamp");
        } else if (flag == "looson") {
            // Execute looson script after publishing waypoint
            executeScript("looson");
        } else if (flag == "land") {
            // Set the landing trigger after 5 seconds
            RCLCPP_INFO(this->get_logger(), "Land flag received, setting land trigger in 5 seconds");
            std::thread([this]() {
                std::this_thread::sleep_for(std::chrono::seconds(5));
                land_triggered_ = true;
                RCLCPP_INFO(this->get_logger(), "Land trigger set to true");
            }).detach();
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<WaypointManager>();
    node->startWaypointSequence();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}
