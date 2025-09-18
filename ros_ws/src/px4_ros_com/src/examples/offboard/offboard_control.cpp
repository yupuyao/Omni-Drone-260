/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/takeoff_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <fcntl.h>      
#include <sys/stat.h>    
#include <sys/mman.h>
#include <unistd.h>  
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <quadrotor_msgs/quadrotor_msgs/msg/position_command.hpp>

#include <chrono>
#include <iostream>
#include <mutex> 

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl(std::atomic<bool>* land_triggered) : Node("offboard_control")
    {
        rclcpp::QoS publisher_qos(1);
        publisher_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
        publisher_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        publisher_qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
        
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
        
        auto qos_ros2 = rclcpp::QoS(
            rclcpp::KeepLast(10))
            .reliable()      
            .keep_last(10);
        
        land_triggered_ = land_triggered;

        //publish topic
        offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>(
            "/fmu/in/offboard_control_mode", publisher_qos);
        trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", publisher_qos);
        vehicle_command_publisher_ = this->create_publisher<VehicleCommand>(
            "/fmu/in/vehicle_command", publisher_qos);
        
        visual_odometry_publisher_ = this->create_publisher<VehicleOdometry>(
            "/fmu/in/vehicle_visual_odometry", publisher_qos);
        
        
        vio_odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/vins_estimator/odometry", 5);
            
        //subscribe topic
        vehicle_local_position_subscriber_ = this->create_subscription<VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position",
            qos,
            std::bind(&OffboardControl::vehicle_local_position_callback, this, std::placeholders::_1)
        );
        
        vehicle_status_subscriber_ = this->create_subscription<VehicleStatus>(
            "/fmu/out/vehicle_status",
            qos,
            std::bind(&OffboardControl::vehicle_status_callback, this, std::placeholders::_1)
        );
        
        odom_subscriber_ = this->create_subscription<VehicleOdometry>(
            "/fmu/out/vehicle_odometry",
            qos,
            std::bind(&OffboardControl::odom_callback, this, std::placeholders::_1)
        );
        
        takeoff_status_subscriber_ = this->create_subscription<TakeoffStatus>(
            "/fmu/out/takeoff_status",
            qos,
            std::bind(&OffboardControl::takeoff_callback, this, std::placeholders::_1)
        );
        
        
        
        vio_odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/orbslam/odom",
            10,
            std::bind(&OffboardControl::vio_odometry_callback, this, std::placeholders::_1)
        );
        
        

        trajectory_subscriber_ = this->create_subscription<quadrotor_msgs::msg::PositionCommand>(
            "/drone_0_planning/pos_cmd",
           10,
            std::bind(&OffboardControl::trajectory_callback, this, std::placeholders::_1)
        );
        
        attitude_subscription_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
        "/fmu/out/vehicle_attitude",
        qos,
        std::bind(&OffboardControl::attitude_callback, this, std::placeholders::_1)
        );
        

        offboard_setpoint_counter_ = 0;
        has_taken_off_.store(false);
        has_land_=false;

        auto timer_callback = [this]() -> void {
            if (offboard_setpoint_counter_ == 10) {
                // Change to Offboard mode after 10 setpoints
                this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

                // Arm the vehicle
                this->arm();
                int result = system("/home/orangepi/utils/hc12");
                //this->takeoff();
                //init_time_ = std::chrono::steady_clock::now();
                //this->disarm();
                //exit(0);
            }

            // offboard_control_mode needs to be paired with trajectory_setpoint
            /*
            if (has_taken_off_.load()){
            publish_offboard_control_mode();
            publish_trajectory_setpoint();
            }
            */
            if (!land_triggered_->load()&&has_received_attitude_){
            publish_offboard_control_mode();
            publish_trajectory_setpoint();
            }
            
            else if (!has_land_&&has_taken_off_.load()){
                rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
                auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
                land_detect_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleLandDetected>(
                    "/fmu/out/vehicle_land_detected",qos,
                    std::bind(&OffboardControl::land_detect_callback, this, std::placeholders::_1)
                );
                land();
                has_land_=true;
            }

            // stop the counter after reaching 11
            if (offboard_setpoint_counter_ < 11) {
                offboard_setpoint_counter_++;
            }
        };
        timer_ = this->create_wall_timer(100ms, timer_callback);
        
        has_received_local_position_ = false;
        has_received_vehicle_status_ = false;
        has_received_trajectory_ = false;
        has_received_vio_odometry_ = false;
        vehicle_local_position_ = px4_msgs::msg::VehicleLocalPosition();
        vehicle_status_ = px4_msgs::msg::VehicleStatus();
        latest_trajectory_ = quadrotor_msgs::msg::PositionCommand();
    }

    void arm();
    void check();
    void disarm();
    void land(); 
    void takeoff();

private:
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<VehicleOdometry>::SharedPtr visual_odometry_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr vio_odometry_publisher_;
    
    rclcpp::Subscription<VehicleAttitude>::SharedPtr attitude_subscription_;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr vehicle_local_position_subscriber_;
    rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_subscriber_;
    rclcpp::Subscription<VehicleOdometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr trajectory_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vio_odometry_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_detect_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::TakeoffStatus>::SharedPtr takeoff_status_subscriber_;
    rclcpp::TimerBase::SharedPtr hover_timer_;

    std::atomic<uint64_t> timestamp_;   //!< common synced timestamped
    std::atomic<bool> has_taken_off_;
    std::atomic<bool>* land_triggered_;

    uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent
    
    VehicleLocalPosition vehicle_local_position_;
    VehicleStatus vehicle_status_;
    VehicleOdometry latest_vehicle_odometry_;
    VehicleAttitude latest_attitude_;
    quadrotor_msgs::msg::PositionCommand latest_trajectory_;
    nav_msgs::msg::Odometry latest_vio_odometry_;
    
    px4_msgs::msg::VehicleLandDetected land_status_;
    px4_msgs::msg::TakeoffStatus takeoff_status_;
    std::mutex data_mutex_; 
    
    bool has_received_local_position_;
    bool has_received_attitude_;
    bool has_received_vehicle_status_;
    bool has_received_trajectory_;
    bool has_received_vio_odometry_;
    bool has_received_odometry_;
    bool has_received_land_status_;
    bool has_land_;
    bool initial_yaw_set_ = false;
    double initial_yaw_ = 0.0;

    void publish_offboard_control_mode();
    void publish_trajectory_setpoint();
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    void publish_visual_odometry();
    void publish_odometry();
    
    void attitude_callback(const VehicleAttitude::SharedPtr msg);
    void vehicle_local_position_callback(const VehicleLocalPosition::SharedPtr msg);
    void vehicle_status_callback(const VehicleStatus::SharedPtr msg);
    void trajectory_callback(const quadrotor_msgs::msg::PositionCommand::SharedPtr msg);
    void land_detect_callback(const VehicleLandDetected::SharedPtr msg);
    void odom_callback(const VehicleOdometry::SharedPtr msg);
    void vio_odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void takeoff_callback(const TakeoffStatus::SharedPtr msg);
    void check_hover_and_land();
    std::chrono::steady_clock::time_point takeoff_time_;
    std::chrono::steady_clock::time_point init_time_;
    
    builtin_interfaces::msg::Time convert_px4_timestamp(uint64_t timestamp_us);
};

void OffboardControl::attitude_callback(const VehicleAttitude::SharedPtr msg)
{
    if (!initial_yaw_set_) {
        latest_attitude_ = *msg;
        RCLCPP_INFO(this->get_logger(), "init");
        }
    has_received_attitude_ = true;
}

void OffboardControl::vehicle_local_position_callback(const VehicleLocalPosition::SharedPtr msg)
{
    vehicle_local_position_ = *msg;
    has_received_local_position_ = true;
}

void OffboardControl::vehicle_status_callback(const VehicleStatus::SharedPtr msg)
{
    vehicle_status_ = *msg;
    has_received_vehicle_status_ = true;
    
    if (msg->pre_flight_checks_pass) {
        RCLCPP_INFO(this->get_logger(), "Preflight checks PASSED.");
    } else {
        RCLCPP_WARN(this->get_logger(), "Preflight checks FAILED.");
    }
}

void OffboardControl::land_detect_callback(const VehicleLandDetected::SharedPtr msg)
{
    land_status_ = *msg;
    has_received_land_status_ = true;
    if (land_status_.landed){
                std::this_thread::sleep_for(std::chrono::seconds(1));

                // Execute the script /home/orangepi/utils/hc12
                int result = system("/home/orangepi/utils/hc12");
                if (result == 0) {
                    RCLCPP_INFO(this->get_logger(), "Successfully send message");
                } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to send message");
                }
                exit(0);
                }
}


void OffboardControl::takeoff_callback(const TakeoffStatus::SharedPtr msg)
{    
    rclcpp::Time now_time = this->now();
    auto now = std::chrono::steady_clock::now();
    //auto elap = std::chrono::duration_cast<std::chrono::seconds>(now - init_time_).count();
    takeoff_status_ = *msg;
    
    // Map takeoff_state to a human-readable string for debugging
    std::string state_description;
    switch (takeoff_status_.takeoff_state) {   
        case 0: state_description = "Disarmed"; break;
        case 1: state_description = "Spooling"; break;
        case 2: state_description = "Ready for takeoff"; break;
        case 3: state_description = "Ramping up"; break;
        case 4: state_description = "In air (ascending)"; break;
        case 5: {state_description = "In air (takeoff complete)";
        has_taken_off_.store(true);
        } break;
        default: state_description = "Unknown"; break;
    }
    
    // Log the current takeoff state
    RCLCPP_INFO(this->get_logger(), "Takeoff state: %d (%s)", 
                takeoff_status_.takeoff_state, state_description.c_str());
}

void OffboardControl::check_hover_and_land()
{
    if (has_taken_off_.load()) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - takeoff_time_).count();

        if (elapsed >= 6) {
            RCLCPP_INFO(this->get_logger(), "Hover complete. Initiating landing.");
            land_triggered_->store(true);
        }
    }
}

void OffboardControl::odom_callback(const VehicleOdometry::SharedPtr msg)
{
    latest_vehicle_odometry_ = *msg;
    has_received_odometry_ = true;
    publish_odometry();
}

void OffboardControl::vio_odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    latest_vio_odometry_ = *msg;
    has_received_vio_odometry_ = true;
    publish_visual_odometry();
}

void OffboardControl::trajectory_callback(const quadrotor_msgs::msg::PositionCommand::SharedPtr msg)
{
    latest_trajectory_ = *msg;
    has_received_trajectory_ = true;
    publish_trajectory_setpoint();
}


/**
 * @brief Send a command to take off the vehicle
 */
void OffboardControl::takeoff()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF);

    RCLCPP_INFO(this->get_logger(), "Take off command send");
}

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

    RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

    RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Send a command to switch to Land mode
 */
void OffboardControl::land()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
    
    RCLCPP_INFO(this->get_logger(), "Switching to land mode");
}

/**
 * @brief Convert PX4 timestamp to ROS2 Time
 */
builtin_interfaces::msg::Time OffboardControl::convert_px4_timestamp(uint64_t timestamp_us)
{
    builtin_interfaces::msg::Time time_msg;
    
    // Convert microseconds to seconds and nanoseconds
    time_msg.sec = static_cast<int32_t>(timestamp_us / 1000000);
    time_msg.nanosec = static_cast<uint32_t>((timestamp_us % 1000000) * 1000);
    
    return time_msg;
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode()
{
    OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
    //RCLCPP_INFO(this->get_logger(), "Published OffboardControlMode");
}

/**
 * @brief Publish a trajectory setpoint
 *        Now uses data from planning trajectory if available
 */
 
void OffboardControl::publish_visual_odometry()
{
    if (!has_received_vio_odometry_) {
        return; 
    }

    VehicleOdometry msg{};
    Eigen::Quaterniond q_eigen(
    latest_attitude_.q[0], 
    latest_attitude_.q[1], 
    latest_attitude_.q[2], 
    latest_attitude_.q[3]
    );
    
    Eigen::Vector3d pos_ned(
    vehicle_local_position_.x,
    vehicle_local_position_.y,
    vehicle_local_position_.z
    );
    
    Eigen::Quaterniond q_frd = q_eigen.conjugate();
    
    Eigen::Affine3d transform(q_frd);
    
    Eigen::Vector3d pos_frd = transform * pos_ned;
    
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.timestamp_sample = msg.timestamp;
    
    msg.pose_frame = VehicleOdometry::POSE_FRAME_FRD;
    
    double scale = -q_frd.z() / latest_vio_odometry_.pose.pose.position.z;
    

    msg.position[0] = latest_vio_odometry_.pose.pose.position.x * scale;
    msg.position[1] = -latest_vio_odometry_.pose.pose.position.y * scale;
    msg.position[2] = -q_frd.z();
    
    double qx = latest_vio_odometry_.pose.pose.orientation.x;
    double qy = latest_vio_odometry_.pose.pose.orientation.y;
    double qz = latest_vio_odometry_.pose.pose.orientation.z;
    double qw = latest_vio_odometry_.pose.pose.orientation.w;
    
    msg.q[0] = qw;
    msg.q[1] = qx;
    msg.q[2] = -qy;
    msg.q[3] = -qz;
    
    /*
    msg.velocity[0] = latest_vio_odometry_.twist.twist.linear.x;
    msg.velocity[1] = -latest_vio_odometry_.twist.twist.linear.y;
    msg.velocity[2] = -latest_vio_odometry_.twist.twist.linear.z;
    
    msg.angular_velocity[0] = latest_vio_odometry_.twist.twist.angular.x; 
    msg.angular_velocity[1] = -latest_vio_odometry_.twist.twist.angular.y;
    msg.angular_velocity[2] = -latest_vio_odometry_.twist.twist.angular.z;
    
    for (int i = 0; i < 3; i++) {
        msg.position_variance[i] = latest_vio_odometry_.pose.covariance[i*6 + i];
        
        msg.velocity_variance[i] = latest_vio_odometry_.twist.covariance[i*6 + i];
        
        msg.orientation_variance[i] = latest_vio_odometry_.pose.covariance[(i+3)*6 + (i+3)];
    }
    */
    
    visual_odometry_publisher_->publish(msg);
    RCLCPP_DEBUG(this->get_logger(), "Published visual odometry data");
}

void OffboardControl::publish_odometry()
{
    if (!has_received_odometry_) {
        return; 
    }
    
    Eigen::Quaterniond q_eigen(
    latest_attitude_.q[0], 
    latest_attitude_.q[1], 
    latest_attitude_.q[2], 
    latest_attitude_.q[3]
    );
    
    Eigen::Quaterniond q_frd = q_eigen.conjugate();
    
    Eigen::Affine3d transform(q_frd);
    
    nav_msgs::msg::Odometry vio_msg{};
    
    vio_msg.header.stamp.nanosec = (latest_vehicle_odometry_.timestamp % 1000000) * 1000;
    vio_msg.header.stamp.sec = latest_vehicle_odometry_.timestamp / 1000000;
    vio_msg.header.frame_id = "odom";
    
    Eigen::Vector3d pos_ned(
    latest_vehicle_odometry_.position[0],
    latest_vehicle_odometry_.position[1],
    latest_vehicle_odometry_.position[2]
    );
    Eigen::Vector3d pos_frd = transform * pos_ned;

    vio_msg.pose.pose.position.x = pos_frd.x();
    vio_msg.pose.pose.position.y = -pos_frd.y();
    vio_msg.pose.pose.position.z = -pos_frd.z();


    vio_msg.pose.pose.orientation.w = q_frd.w();
    vio_msg.pose.pose.orientation.x = q_frd.x();
    vio_msg.pose.pose.orientation.y = -q_frd.y();
    vio_msg.pose.pose.orientation.z = -q_frd.z();
    
    Eigen::Vector3d vel_ned(
    latest_vehicle_odometry_.velocity[0],
    latest_vehicle_odometry_.velocity[1],
    latest_vehicle_odometry_.velocity[2]
    );
    Eigen::Vector3d vel_frd = transform.rotation() * vel_ned;

    vio_msg.twist.twist.linear.x = vel_frd.x();
    vio_msg.twist.twist.linear.y = -vel_frd.y();
    vio_msg.twist.twist.linear.z = -vel_frd.z();

    Eigen::Vector3d angvel_ned(
    latest_vehicle_odometry_.angular_velocity[0],
    latest_vehicle_odometry_.angular_velocity[1],
    latest_vehicle_odometry_.angular_velocity[2]
    );
    Eigen::Vector3d angvel_frd = transform.rotation() * angvel_ned;

    vio_msg.twist.twist.angular.x = angvel_frd.x();
    vio_msg.twist.twist.angular.y = -angvel_frd.y();
    vio_msg.twist.twist.angular.z = -angvel_frd.z();
    

    for (int i = 0; i < 3; i++) {
        vio_msg.pose.covariance[i*6 + i] = latest_vehicle_odometry_.position_variance[i];
    }
    
    for (int i = 0; i < 3; i++) {
        vio_msg.pose.covariance[(i+3)*6 + (i+3)] = latest_vehicle_odometry_.orientation_variance[i];
    }
    
    for (int i = 0; i < 3; i++) {
        vio_msg.twist.covariance[i*6 + i] = latest_vehicle_odometry_.velocity_variance[i];
    }
    

    vio_odometry_publisher_->publish(vio_msg);
    RCLCPP_DEBUG(this->get_logger(), "Published VehicleOdometry data to VIO format");
}

void OffboardControl::publish_trajectory_setpoint()
{
    TrajectorySetpoint msg{};
    
    Eigen::Quaterniond q_eigen(
    latest_attitude_.q[0], 
    latest_attitude_.q[1], 
    latest_attitude_.q[2], 
    latest_attitude_.q[3]
    );
    
    Eigen::Affine3d transform(q_eigen);

    if (has_received_trajectory_) {     
        
        double yaw_ego = -latest_trajectory_.yaw;
        Eigen::Vector3d heading_frd(cos(yaw_ego), sin(yaw_ego), 0.0); 
        
        Eigen::Vector3d pos_frd(
        latest_trajectory_.position.x, 
        -latest_trajectory_.position.y, 
        -latest_trajectory_.position.z
        );
        
        Eigen::Vector3d vel_frd(
        latest_trajectory_.velocity.x, 
        -latest_trajectory_.velocity.y, 
        -latest_trajectory_.velocity.z
        );
        
        Eigen::Vector3d pos_ned = transform * pos_frd;
        Eigen::Vector3d vel_ned = transform.rotation() * vel_frd;
        
        msg.position[0] = pos_ned.x();
        msg.position[1] = pos_ned.y();
        msg.position[2] = pos_ned.z();
        
        msg.velocity[0] = vel_ned.x();
        msg.velocity[1] = vel_ned.y();
        msg.velocity[2] = vel_ned.z();
        
        Eigen::Vector3d heading_ned = q_eigen * heading_frd;
        msg.yaw = std::atan2(heading_ned.y(), heading_ned.x());
        
        //msg.acceleration[0] = latest_trajectory_.acceleration.x;
        //msg.acceleration[1] = -latest_trajectory_.acceleration.y;
        //msg.acceleration[2] = -latest_trajectory_.acceleration.z;
        
        
        //if (msg.yaw > M_PI) msg.yaw -= 2.0 * M_PI;
        //if (msg.yaw < -M_PI) msg.yaw += 2.0 * M_PI;
        
        RCLCPP_INFO(this->get_logger(), "Publishing trajectory setpoint from planning data");
    }
    else if (!has_taken_off_.load()) {
    
        Eigen::Vector3d pos_frd(
        0.0, 
        0.0, 
        -1.0
        );
        
        Eigen::Vector3d pos_ned = pos_frd;

        msg.position[0] = pos_ned.x();
        msg.position[1] = pos_ned.y();
        msg.position[2] = pos_ned.z();


        //msg.acceleration[0] = 0.0;
        //msg.acceleration[1] = 0.0;
        //msg.acceleration[2] = 0.0;
        if (!initial_yaw_set_) {
        double yaw = 0.0; 
        Eigen::Vector3d heading_frd(cos(yaw), sin(yaw), 0.0);
        Eigen::Vector3d heading_ned = q_eigen * heading_frd;
        initial_yaw_ = std::atan2(heading_ned.y(), heading_ned.x());
        initial_yaw_set_ = true;
        RCLCPP_INFO(this->get_logger(), "init yaw", msg.yaw);
        }
        msg.yaw = initial_yaw_;

        //RCLCPP_DEBUG(this->get_logger(), "Publishing default trajectory setpoint");
        RCLCPP_INFO(this->get_logger(), "takeoff point, yaw = %.3f", msg.yaw);
    }   
    /*else {

        msg.position[0] = vehicle_local_position_.x;
        msg.position[1] = vehicle_local_position_.y;
        msg.position[2] = vehicle_local_position_.z;

        msg.velocity[0] = 0.0;
        msg.velocity[1] = 0.0;
        msg.velocity[2] = 0.0;

        //msg.acceleration[0] = 0.0;
        //msg.acceleration[1] = 0.0;
        //msg.acceleration[2] = 0.0;

        msg.yaw = vehicle_local_position_.heading;
        //RCLCPP_DEBUG(this->get_logger(), "Publishing default trajectory setpoint");
        RCLCPP_INFO(this->get_logger(), "Hold");
    }*/
    else {

        Eigen::Vector3d pos_frd(
        0.0, 
        0.0, 
        -1.0
        );
        
        Eigen::Vector3d pos_ned = pos_frd;

        msg.position[0] = pos_ned.x();
        msg.position[1] = pos_ned.y();
        msg.position[2] = pos_ned.z();

        //msg.velocity[0] = 0.0;
        //msg.velocity[1] = 0.0;
        //msg.velocity[2] = 0.0;

        //msg.acceleration[0] = 0.0;
        //msg.acceleration[1] = 0.0;
        //msg.acceleration[2] = 0.0;
        msg.yaw = initial_yaw_;

        //msg.yaw = 0.0;
        //RCLCPP_DEBUG(this->get_logger(), "Publishing default trajectory setpoint");
        RCLCPP_INFO(this->get_logger(), "Hold");
    }
    
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
    //RCLCPP_INFO(this->get_logger(), "Published TrajectorySetpoint: pos=[%.2f, %.2f, %.2f], yaw=%.2f",
    //            msg.position[0], msg.position[1], msg.position[2], msg.yaw);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
    VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_publisher_->publish(msg);
}

int main(int argc, char *argv[])
{
    const char* shm_name = "/shared_mem";

    int shm_fd = shm_open(shm_name, O_CREAT | O_RDWR, S_IRUSR | S_IWUSR);
    if (shm_fd == -1) {
        std::cerr << "Failed to open shared memory" << std::endl;
    }

    if (ftruncate(shm_fd, sizeof(std::atomic<bool>)) == -1) {
        std::cerr << "Failed to set size of shared memory" << std::endl;
    }

    void* shared_mem = mmap(nullptr, sizeof(std::atomic<bool>), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (shared_mem == MAP_FAILED) {
        std::cerr << "Failed to map shared memory" << std::endl;
    }

    auto* land_triggered_ = new(shared_mem) std::atomic<bool>(false);
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>(land_triggered_));

    rclcpp::shutdown();
    return 0;
}
