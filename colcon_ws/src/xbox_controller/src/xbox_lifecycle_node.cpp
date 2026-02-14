#include <cstdio>

#include "xbox_controller/xbox_lifecycle_node.hpp"

XboxLifecycleNode::XboxLifecycleNode()
  : LifecycleNode("xbox_node")
{
  this->trigger_transition(
    lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE); 
  this->trigger_transition(
    lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE); 
}

XboxLifecycleNode::~XboxLifecycleNode() 
{
}

CallbackReturn XboxLifecycleNode::on_configure(const rclcpp_lifecycle::State &)
{
  joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>("xbox_joy", 10);
  client_ = this->create_client<std_srvs::srv::Trigger>("xbox_publish_activate");
  service_ = this->create_service<std_srvs::srv::Trigger>("xbox_publish_activate",
    std::bind(&XboxLifecycleNode::handle_activate_request,
    this, std::placeholders::_1, std::placeholders::_2));

  // construct xbox event hook
  controller_.set_callback(
    std::bind(&XboxLifecycleNode::on_xbox_event, this, std::placeholders::_1));
  
  // controller spinning start (don't destroy for other states except for shutdown)
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&XboxLifecycleNode::timer_callback, this));

  RCLCPP_INFO(get_logger(), "Configured");
  return CallbackReturn::SUCCESS;
}

CallbackReturn XboxLifecycleNode::on_activate(const rclcpp_lifecycle::State &)
{
  joy_pub_->on_activate();
  // timer_->reset();
  RCLCPP_INFO(get_logger(), "Activated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn XboxLifecycleNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  joy_pub_->on_deactivate();
  // timer_->cancel();
  RCLCPP_INFO(get_logger(), "Deactivated");
  return CallbackReturn::SUCCESS;
}

// CallbackReturn XboxLifecycleNode::on_cleanup(const rclcpp_lifecycle::State &)
// {
//   timer_.reset();
//   RCLCPP_INFO(get_logger(), "Cleanup");
//   return CallbackReturn::SUCCESS;
// }

CallbackReturn XboxLifecycleNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  client_.reset();
  service_.reset();
  timer_.reset();
  RCLCPP_INFO(get_logger(), "Shutdown");
  return CallbackReturn::SUCCESS;
}

void XboxLifecycleNode::on_xbox_event(const XboxState& state)
{
  // (top) publishing only happens if node is active
  if (this->get_current_state().id() ==
    lifecycle_msgs::msg:: State::PRIMARY_STATE_ACTIVE)
  {
    publish_joy(state);
  }

  // check mode change
  this->on_check_mode_trigger(state.auto_mode);
  if (change_mode_)
  {
    RCLCPP_DEBUG(get_logger(), "mode changed");
    // executes once for change
    if (!client_->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_WARN(this->get_logger(), "Service not available");
      return;
    }
      
    // change tranistion requested
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    client_->async_send_request(request,
      std::bind(&XboxLifecycleNode::response_callback, this, std::placeholders::_1));

    // awake user on manual mode!
    if (!state.auto_mode)
    {
      RCLCPP_INFO(get_logger(), "Manual Control Start");
      controller_.vibrate_motion();
    } else {
      RCLCPP_WARN(get_logger(), "Auto Mode Start");
    }
    change_mode_ = false;
  }
  
  // update mode history
  prev_mode_ = state.auto_mode;
}

void XboxLifecycleNode::handle_activate_request(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (this->get_current_state().id() ==
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    this->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);

    response->success = true;
    response->message = "Deactivated for xbox publishing";
    RCLCPP_DEBUG(get_logger(), "Service called to deactivate");
  }
  else if (this->get_current_state().id() ==
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    this->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    response->success = true;
    response->message = "Activated for xbox publishing";
    RCLCPP_DEBUG(get_logger(), "Service called to activate");
  }
  else
  {
    response->success = false;
    response->message = "no matching transition to perform";
    RCLCPP_ERROR(get_logger(), "Service called to none!");
  }
}

void XboxLifecycleNode::publish_joy(const XboxState& state)
{
  auto msg = sensor_msgs::msg::Joy();
  msg.axes = {state.lx, state.ly, state.rx, state.ry};
  msg.buttons = {state.auto_mode, state.b_pressed};

  joy_pub_->publish(msg);
}

void XboxLifecycleNode::timer_callback()
{
  controller_.poll();
}

void XboxLifecycleNode::response_callback(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
{
  auto response = future.get();

  if (response->success)
  {
    RCLCPP_DEBUG(this->get_logger(),
      "Service succeeded: %s",
      response->message.c_str());
  }
  else
  {
    RCLCPP_WARN(this->get_logger(),
      "Service failed: %s",
      response->message.c_str());
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<XboxLifecycleNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}