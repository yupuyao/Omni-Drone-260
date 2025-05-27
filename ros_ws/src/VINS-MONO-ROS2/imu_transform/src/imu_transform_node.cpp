#include "imu_transform/imu_transform_node.hpp"

namespace imu_transform {

ImuTransformNode::ImuTransformNode(const rclcpp::NodeOptions& options)
    : Node("imu_transform_node", options)
{
    // Declare and get parameters
    this->declare_parameter("imu_frame_id", "imu_link");
    this->declare_parameter("base_frame_id", "base_link");
    this->declare_parameter("publish_tf", true);
    
    imu_frame_id_ = this->get_parameter("imu_frame_id").as_string();
    base_frame_id_ = this->get_parameter("base_frame_id").as_string();
    publish_tf_ = this->get_parameter("publish_tf").as_bool();
    
    // Create QoS profile according to PX4 standards
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
        .best_effort()
        .durability_volatile();
    
    // Initialize publisher
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(
        "imu/data", 10);
    
    // Initialize TF broadcaster if enabled
    if (publish_tf_) {
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
    
    // Initialize subscribers
    sensor_subscription_ = this->create_subscription<px4_msgs::msg::SensorCombined>(
        "/fmu/out/sensor_combined",
        qos,
        std::bind(&ImuTransformNode::sensor_callback, this, std::placeholders::_1)
    );
    
    attitude_subscription_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
        "/fmu/out/vehicle_attitude",
        qos,
        std::bind(&ImuTransformNode::attitude_callback, this, std::placeholders::_1)
    );
    
    RCLCPP_INFO(this->get_logger(), "IMU Transform Node initialized");
}

void ImuTransformNode::sensor_callback(const px4_msgs::msg::SensorCombined::SharedPtr msg)
{
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_sensor_ = msg;
    }
    
    px4_msgs::msg::VehicleAttitude::SharedPtr attitude;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!latest_attitude_) return;
        attitude = latest_attitude_;
    }
    
    auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();
    imu_msg->header.stamp = convert_px4_timestamp(msg->timestamp);
    imu_msg->header.frame_id = imu_frame_id_;
    
    // Copy angular velocity
    imu_msg->angular_velocity.x = msg->gyro_rad[0];
    imu_msg->angular_velocity.y = msg->gyro_rad[1];
    imu_msg->angular_velocity.z = msg->gyro_rad[2];
    
    // Copy linear acceleration
    imu_msg->linear_acceleration.x = msg->accelerometer_m_s2[0];
    imu_msg->linear_acceleration.y = msg->accelerometer_m_s2[1];
    imu_msg->linear_acceleration.z = msg->accelerometer_m_s2[2];
    
    // Copy orientation quaternion
    imu_msg->orientation.w = attitude->q[0];
    imu_msg->orientation.x = attitude->q[1];
    imu_msg->orientation.y = attitude->q[2];
    imu_msg->orientation.z = attitude->q[3];
    
    // Set covariances
    std::array<double, 9> orientation_cov = {
        0.018832744466606915, 0.0, 0.0,
        0.0, 0.19175511332543127, 0.0,
        0.0, 0.0, 0.18804607625176392
    };
    
    std::array<double, 9> angular_velocity_cov = {
        0.018832744466606915, 0.0, 0.0,
        0.0, 0.19175511332543127, 0.0,
        0.0, 0.0, 0.18804607625176392
    };
    
    std::array<double, 9> linear_acceleration_cov = {
        0.018832744466606915, 0.0, 0.0,
        0.0, 0.19175511332543127, 0.0,
        0.0, 0.0, 0.18804607625176392
    };
    
    for (size_t i = 0; i < 9; i++) {
        imu_msg->orientation_covariance[i] = orientation_cov[i];
        imu_msg->angular_velocity_covariance[i] = angular_velocity_cov[i];
        imu_msg->linear_acceleration_covariance[i] = linear_acceleration_cov[i];
    }
    
    // Publish IMU message
    imu_publisher_->publish(std::move(imu_msg));
    
    // Publish TF if enabled
    if (publish_tf_) {
        publish_imu_tf(convert_px4_timestamp(msg->timestamp), attitude);
    }
}

void ImuTransformNode::attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg) 
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_attitude_ = msg;
}

builtin_interfaces::msg::Time ImuTransformNode::convert_px4_timestamp(uint64_t timestamp_us)
{
    builtin_interfaces::msg::Time time_msg;
    
    // Convert microseconds to seconds and nanoseconds
    time_msg.sec = static_cast<int32_t>(timestamp_us / 1000000);
    time_msg.nanosec = static_cast<uint32_t>((timestamp_us % 1000000) * 1000);
    
    return time_msg;
}

void ImuTransformNode::publish_imu_tf(const rclcpp::Time& stamp, const px4_msgs::msg::VehicleAttitude::SharedPtr attitude) 
{
    if (!tf_broadcaster_) return;
    
    geometry_msgs::msg::TransformStamped transform;
    
    transform.header.stamp = stamp;
    transform.header.frame_id = base_frame_id_;
    transform.child_frame_id = imu_frame_id_;
    
    // Set translation (assuming IMU is at the origin of the base_link frame)
    transform.transform.translation.x = 0.0;
    transform.transform.translation.y = 0.0;
    transform.transform.translation.z = 0.0;
    
    // Set rotation from attitude
    transform.transform.rotation.w = attitude->q[0];
    transform.transform.rotation.x = attitude->q[1];
    transform.transform.rotation.y = attitude->q[2];
    transform.transform.rotation.z = attitude->q[3];
    
    tf_broadcaster_->sendTransform(transform);
}

} // namespace imu_transform

// Standalone node main function
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<imu_transform::ImuTransformNode>());
    rclcpp::shutdown();
    return 0;
}