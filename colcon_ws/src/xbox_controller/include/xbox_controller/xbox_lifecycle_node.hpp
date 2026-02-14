#ifndef WINDOW_ROS2_XBOX_CONTROLLER_NODE_HPP_
#define WINDOW_ROS2_XBOX_CONTROLLER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "xbox_controller/xbox_joystick.hpp"

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class XboxLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  XboxLifecycleNode();
  ~XboxLifecycleNode();
  
protected:
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
  
private:
  /// @brief callback hook for controller event (tied to timer callback)
  void on_xbox_event(const XboxState& state);

  /// @brief changing transition state triggered by controller
  void handle_activate_request(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  
  /// @brief publish ros2 sensor msg joy
  void publish_joy(const XboxState& state);

  /// @brief controller spin
  void timer_callback();

  /// @brief client response callback
  void response_callback(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future);

  /// @brief mode change checker
  inline void on_check_mode_trigger(bool current)
  {
    if (current != prev_mode_) {
      change_mode_ = true;
    }
  }

  /// @brief xbox joystick controller, ros independent
  XboxJoystick controller_;
  /// @brief change mode (edge detection)
  bool change_mode_{false}, prev_mode_{false};
  /// @brief joystick sensor msg publisher
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
  /// @brief service for publisher trigger
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
  /// @brief client for publisher trigger
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
  /// @brief timer to recieve xbox values
  rclcpp::TimerBase::SharedPtr timer_;
};
#endif /* WINDOW_ROS2_XBOX_CONTROLLER_NODE_HPP_ */