#ifndef IMU_TRANSFORM_NODE_HPP
#define IMU_TRANSFORM_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <mutex>
#include <memory>

namespace imu_transform {

class ImuTransformNode : public rclcpp::Node {
public:
    explicit ImuTransformNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    virtual ~ImuTransformNode() = default;

private:
    // Callback functions
    void sensor_callback(const px4_msgs::msg::SensorCombined::SharedPtr msg);
    void attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);
    
    // Helper function to convert PX4 timestamp to ROS time
    builtin_interfaces::msg::Time convert_px4_timestamp(uint64_t timestamp_us);
    
    // Publish TF transform for IMU frame
    void publish_imu_tf(const rclcpp::Time& stamp, const px4_msgs::msg::VehicleAttitude::SharedPtr attitude);
    
    // Mutex for thread safety when accessing shared data
    std::mutex data_mutex_;
    
    // Store the latest sensor and attitude messages
    px4_msgs::msg::SensorCombined::SharedPtr latest_sensor_;
    px4_msgs::msg::VehicleAttitude::SharedPtr latest_attitude_;
    
    // Publishers and subscribers
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr sensor_subscription_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_subscription_;
    
    // TF broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // Parameters
    std::string imu_frame_id_;
    std::string base_frame_id_;
    bool publish_tf_;
};

} // namespace imu_transform

#endif // IMU_TRANSFORM_NODE_HPP