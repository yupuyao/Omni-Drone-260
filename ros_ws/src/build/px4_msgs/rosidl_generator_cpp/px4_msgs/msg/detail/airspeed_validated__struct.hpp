// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from px4_msgs:msg/AirspeedValidated.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__AIRSPEED_VALIDATED__STRUCT_HPP_
#define PX4_MSGS__MSG__DETAIL__AIRSPEED_VALIDATED__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__px4_msgs__msg__AirspeedValidated __attribute__((deprecated))
#else
# define DEPRECATED__px4_msgs__msg__AirspeedValidated __declspec(deprecated)
#endif

namespace px4_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct AirspeedValidated_
{
  using Type = AirspeedValidated_<ContainerAllocator>;

  explicit AirspeedValidated_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->timestamp = 0ull;
      this->indicated_airspeed_m_s = 0.0f;
      this->calibrated_airspeed_m_s = 0.0f;
      this->true_airspeed_m_s = 0.0f;
      this->calibrated_ground_minus_wind_m_s = 0.0f;
      this->true_ground_minus_wind_m_s = 0.0f;
      this->airspeed_sensor_measurement_valid = false;
      this->selected_airspeed_index = 0;
      this->airspeed_derivative_filtered = 0.0f;
      this->throttle_filtered = 0.0f;
      this->pitch_filtered = 0.0f;
    }
  }

  explicit AirspeedValidated_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->timestamp = 0ull;
      this->indicated_airspeed_m_s = 0.0f;
      this->calibrated_airspeed_m_s = 0.0f;
      this->true_airspeed_m_s = 0.0f;
      this->calibrated_ground_minus_wind_m_s = 0.0f;
      this->true_ground_minus_wind_m_s = 0.0f;
      this->airspeed_sensor_measurement_valid = false;
      this->selected_airspeed_index = 0;
      this->airspeed_derivative_filtered = 0.0f;
      this->throttle_filtered = 0.0f;
      this->pitch_filtered = 0.0f;
    }
  }

  // field types and members
  using _timestamp_type =
    uint64_t;
  _timestamp_type timestamp;
  using _indicated_airspeed_m_s_type =
    float;
  _indicated_airspeed_m_s_type indicated_airspeed_m_s;
  using _calibrated_airspeed_m_s_type =
    float;
  _calibrated_airspeed_m_s_type calibrated_airspeed_m_s;
  using _true_airspeed_m_s_type =
    float;
  _true_airspeed_m_s_type true_airspeed_m_s;
  using _calibrated_ground_minus_wind_m_s_type =
    float;
  _calibrated_ground_minus_wind_m_s_type calibrated_ground_minus_wind_m_s;
  using _true_ground_minus_wind_m_s_type =
    float;
  _true_ground_minus_wind_m_s_type true_ground_minus_wind_m_s;
  using _airspeed_sensor_measurement_valid_type =
    bool;
  _airspeed_sensor_measurement_valid_type airspeed_sensor_measurement_valid;
  using _selected_airspeed_index_type =
    int8_t;
  _selected_airspeed_index_type selected_airspeed_index;
  using _airspeed_derivative_filtered_type =
    float;
  _airspeed_derivative_filtered_type airspeed_derivative_filtered;
  using _throttle_filtered_type =
    float;
  _throttle_filtered_type throttle_filtered;
  using _pitch_filtered_type =
    float;
  _pitch_filtered_type pitch_filtered;

  // setters for named parameter idiom
  Type & set__timestamp(
    const uint64_t & _arg)
  {
    this->timestamp = _arg;
    return *this;
  }
  Type & set__indicated_airspeed_m_s(
    const float & _arg)
  {
    this->indicated_airspeed_m_s = _arg;
    return *this;
  }
  Type & set__calibrated_airspeed_m_s(
    const float & _arg)
  {
    this->calibrated_airspeed_m_s = _arg;
    return *this;
  }
  Type & set__true_airspeed_m_s(
    const float & _arg)
  {
    this->true_airspeed_m_s = _arg;
    return *this;
  }
  Type & set__calibrated_ground_minus_wind_m_s(
    const float & _arg)
  {
    this->calibrated_ground_minus_wind_m_s = _arg;
    return *this;
  }
  Type & set__true_ground_minus_wind_m_s(
    const float & _arg)
  {
    this->true_ground_minus_wind_m_s = _arg;
    return *this;
  }
  Type & set__airspeed_sensor_measurement_valid(
    const bool & _arg)
  {
    this->airspeed_sensor_measurement_valid = _arg;
    return *this;
  }
  Type & set__selected_airspeed_index(
    const int8_t & _arg)
  {
    this->selected_airspeed_index = _arg;
    return *this;
  }
  Type & set__airspeed_derivative_filtered(
    const float & _arg)
  {
    this->airspeed_derivative_filtered = _arg;
    return *this;
  }
  Type & set__throttle_filtered(
    const float & _arg)
  {
    this->throttle_filtered = _arg;
    return *this;
  }
  Type & set__pitch_filtered(
    const float & _arg)
  {
    this->pitch_filtered = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint32_t MESSAGE_VERSION =
    0u;

  // pointer types
  using RawPtr =
    px4_msgs::msg::AirspeedValidated_<ContainerAllocator> *;
  using ConstRawPtr =
    const px4_msgs::msg::AirspeedValidated_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<px4_msgs::msg::AirspeedValidated_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<px4_msgs::msg::AirspeedValidated_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      px4_msgs::msg::AirspeedValidated_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<px4_msgs::msg::AirspeedValidated_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      px4_msgs::msg::AirspeedValidated_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<px4_msgs::msg::AirspeedValidated_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<px4_msgs::msg::AirspeedValidated_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<px4_msgs::msg::AirspeedValidated_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__px4_msgs__msg__AirspeedValidated
    std::shared_ptr<px4_msgs::msg::AirspeedValidated_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__px4_msgs__msg__AirspeedValidated
    std::shared_ptr<px4_msgs::msg::AirspeedValidated_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AirspeedValidated_ & other) const
  {
    if (this->timestamp != other.timestamp) {
      return false;
    }
    if (this->indicated_airspeed_m_s != other.indicated_airspeed_m_s) {
      return false;
    }
    if (this->calibrated_airspeed_m_s != other.calibrated_airspeed_m_s) {
      return false;
    }
    if (this->true_airspeed_m_s != other.true_airspeed_m_s) {
      return false;
    }
    if (this->calibrated_ground_minus_wind_m_s != other.calibrated_ground_minus_wind_m_s) {
      return false;
    }
    if (this->true_ground_minus_wind_m_s != other.true_ground_minus_wind_m_s) {
      return false;
    }
    if (this->airspeed_sensor_measurement_valid != other.airspeed_sensor_measurement_valid) {
      return false;
    }
    if (this->selected_airspeed_index != other.selected_airspeed_index) {
      return false;
    }
    if (this->airspeed_derivative_filtered != other.airspeed_derivative_filtered) {
      return false;
    }
    if (this->throttle_filtered != other.throttle_filtered) {
      return false;
    }
    if (this->pitch_filtered != other.pitch_filtered) {
      return false;
    }
    return true;
  }
  bool operator!=(const AirspeedValidated_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AirspeedValidated_

// alias to use template instance with default allocator
using AirspeedValidated =
  px4_msgs::msg::AirspeedValidated_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint32_t AirspeedValidated_<ContainerAllocator>::MESSAGE_VERSION;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace px4_msgs

#endif  // PX4_MSGS__MSG__DETAIL__AIRSPEED_VALIDATED__STRUCT_HPP_
