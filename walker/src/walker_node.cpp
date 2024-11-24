/**
 * @file walker_node.cpp
 * @author Anirudh Swarankar
 * @brief Souce file for walker node
 * Copyright [2024] [Anirudh Swarankar]

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
 */

#include <walker/walker_node.hpp>

namespace walker {

/**
 * @brief Constructor
 *
 * @param options node options
 */
WalkerNode::WalkerNode(rclcpp::Node::SharedPtr node)
    : node_(node), direction_(Direction::CLOCKWISE) {
  this->declareAndLoadParameter(
      "max_linear_speed", max_linear_vel_param_,
      "Maximum linear velocity the walker will generate", true, false, false,
      0.0, 1.0, 0.1);
  this->declareAndLoadParameter(
      "max_angular_speed", max_angular_vel_param_,
      "Maximum angular velocity the walker will generate", true, false, false,
      0.0, 0.5, 0.1);
  this->setup();
  RCLCPP_INFO(node_->get_logger(), "Initializing robot in IDLE state");
  state_ = std::make_shared<IDLE>();
}

/**
 * @brief Declares and loads a ROS parameter
 *
 * @param name name
 * @param param parameter variable to load into
 * @param description description
 * @param add_to_auto_reconfigurable_params enable reconfiguration of parameter
 * @param is_required whether failure to load parameter will stop node
 * @param read_only set parameter to read-only
 * @param from_value parameter range minimum
 * @param to_value parameter range maximum
 * @param step_value parameter range step
 * @param additional_constraints additional constraints description
 */
template <typename T>
void WalkerNode::declareAndLoadParameter(
    const std::string &name, T &param, const std::string &description,
    const bool add_to_auto_reconfigurable_params, const bool is_required,
    const bool read_only, const std::optional<double> &from_value,
    const std::optional<double> &to_value,
    const std::optional<double> &step_value,
    const std::string &additional_constraints) {
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.description = description;
  param_desc.additional_constraints = additional_constraints;
  param_desc.read_only = read_only;

  auto type = rclcpp::ParameterValue(param).get_type();

  if (from_value.has_value() && to_value.has_value()) {
    if constexpr (std::is_integral_v<T>) {
      rcl_interfaces::msg::IntegerRange range;
      T step = static_cast<T>(step_value.has_value() ? step_value.value() : 1);
      range.set__from_value(static_cast<T>(from_value.value()))
          .set__to_value(static_cast<T>(to_value.value()))
          .set__step(step);
      param_desc.integer_range = {range};
    } else if constexpr (std::is_floating_point_v<T>) {
      rcl_interfaces::msg::FloatingPointRange range;
      T step =
          static_cast<T>(step_value.has_value() ? step_value.value() : 1.0);
      range.set__from_value(static_cast<T>(from_value.value()))
          .set__to_value(static_cast<T>(to_value.value()))
          .set__step(step);
      param_desc.floating_point_range = {range};
    } else {
      RCLCPP_WARN(node_->get_logger(),
                  "Parameter type of parameter '%s' does not support "
                  "specifying a range",
                  name.c_str());
    }
  }

  node_->declare_parameter(name, type, param_desc);

  try {
    param = node_->get_parameter(name).get_value<T>();
    std::stringstream ss;
    ss << "Loaded parameter '" << name << "': ";
    if constexpr (is_vector_v<T>) {
      ss << "[";
      for (const auto &element : param)
        ss << element << (&element != &param.back() ? ", " : "]");
    } else {
      ss << param;
    }
    RCLCPP_INFO_STREAM(node_->get_logger(), ss.str());
  } catch (rclcpp::exceptions::ParameterUninitializedException &) {
    if (is_required) {
      RCLCPP_FATAL_STREAM(node_->get_logger(), "Missing required parameter '"
                                                   << name << "', exiting");
      exit(EXIT_FAILURE);
    } else {
      std::stringstream ss;
      ss << "Missing parameter '" << name << "', using default value: ";
      if constexpr (is_vector_v<T>) {
        ss << "[";
        for (const auto &element : param)
          ss << element << (&element != &param.back() ? ", " : "]");
      } else {
        ss << param;
      }
      RCLCPP_WARN_STREAM(node_->get_logger(), ss.str());
    }
  }

  if (add_to_auto_reconfigurable_params) {
    std::function<void(const rclcpp::Parameter &)> setter =
        [&param](const rclcpp::Parameter &p) { param = p.get_value<T>(); };
    auto_reconfigurable_params_.push_back(std::make_tuple(name, setter));
  }
}

/**
 * @brief Handles reconfiguration when a parameter value is changed
 *
 * @param parameters parameters
 * @return parameter change result
 */
rcl_interfaces::msg::SetParametersResult WalkerNode::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters) {
  for (const auto &param : parameters) {
    for (auto &auto_reconfigurable_param : auto_reconfigurable_params_) {
      if (param.get_name() == std::get<0>(auto_reconfigurable_param)) {
        std::get<1>(auto_reconfigurable_param)(param);
        RCLCPP_INFO(node_->get_logger(), "Reconfigured parameter '%s'",
                    param.get_name().c_str());
        break;
      }
    }
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

/**
 * @brief Sets up subscribers, publishers, etc. to configure the node
 */
void WalkerNode::setup() {
  // callback for dynamic parameter configuration
  parameters_callback_ = node_->add_on_set_parameters_callback(
      std::bind(&WalkerNode::parametersCallback, this, std::placeholders::_1));

  // subscriber for handling incoming messages
  subscriber_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&WalkerNode::laserScanCallback, this, std::placeholders::_1));
  RCLCPP_INFO(node_->get_logger(), "Subscribed to '%s'",
              subscriber_->get_topic_name());

  // publisher for publishing outgoing messages
  publisher_ =
      node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  RCLCPP_INFO(node_->get_logger(), "Publishing to '%s'",
              publisher_->get_topic_name());

  // Service server
  startService_ = node_->create_service<std_srvs::srv::Trigger>(
      "/start", std::bind(&WalkerNode::start_callback, this,
                          std::placeholders::_1, std::placeholders::_2));
  RCLCPP_INFO(node_->get_logger(), "Service server to '%s'",
              startService_->get_service_name());

  // Service server
  stopService_ = node_->create_service<std_srvs::srv::Trigger>(
      "/stop", std::bind(&WalkerNode::end_callback, this, std::placeholders::_1,
                         std::placeholders::_2));
  RCLCPP_INFO(node_->get_logger(), "Service server to '%s'",
              startService_->get_service_name());
}

/**
 * @brief Service callback for stop service
 * @param req
 * @param resp
 */
void WalkerNode::end_callback(
    const std::shared_ptr<std_srvs::srv::Trigger_Request> req,
    std::shared_ptr<std_srvs::srv::Trigger_Response> resp) {
  // change state to idle
  try {
    transition(std::make_shared<IDLE>());
    resp->message = "Changed state to IDLE";
    resp->success = true;
  } catch (const std::exception e) {
    resp->message = "Failed to changes state to IDLE";
    resp->success = false;
  }
}

/**
 * @brief Service callback for start service
 * @param Trigger request, response
 */
void WalkerNode::start_callback(
    const std::shared_ptr<std_srvs::srv::Trigger_Request> req,
    std::shared_ptr<std_srvs::srv::Trigger_Response> resp) {
  // change from idle to runing

  transition(std::make_shared<MoveForward>());
  resp->message = "Changed state to MoveForward from IDLE";
  resp->success = true;
}

/**
 * @brief Main run process of context class
 */
void WalkerNode::run() { state_->execute(this); }

/**
 * @brief Transition handler
 * @param state Next state to transition to
 */
void WalkerNode::transition(std::shared_ptr<State> state) {
  try {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Transition");
    state_ = state;
  } catch (const std::exception e) {
  }
}

/**
 * @brief Function to return the current value of Direction
 * @return Direction 0 CW 1 CCW
 */
Direction &WalkerNode::get_direction() { return direction_; }

/**
 * @brief Function to set the direction
 * @param dir 0 for CW 1 CCW
 * @return true if success
 */
bool WalkerNode::set_direction(Direction dir) {
  direction_ = dir;
  return true;
}

/**
 * @brief Processes messages received by a subscriber
 * @param msg message
 */
void WalkerNode::laserScanCallback(const sensor_msgs::msg::LaserScan &msg) {
  scan_ = msg;
}

/**
 * @brief Implementation of execute function for IDLE concrete class of State
 * @param node
 */
void IDLE::execute(walker::WalkerNode *node) {
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.0;  // Move forward
  cmd.angular.z = 0.0;
  node->publisher_->publish(cmd);
}

/**
 * @brief Implementation of execute function for MoveForward concrete class of
 * State
 * @param node
 */
void MoveForward::execute(WalkerNode *node) {
  float min_range_left = *std::min_element(node->scan_.ranges.begin(),
                                           node->scan_.ranges.begin() + 10);
  float min_range_right = *std::min_element(node->scan_.ranges.end() - 10,
                                            node->scan_.ranges.end());
  if (std::min(min_range_left, min_range_right) < 0.5) {
    node->transition(std::make_shared<Rotate>(node->get_direction()));
  }

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = node->max_linear_vel_param_;  // Move forward
  cmd.angular.z = 0.0;
  node->publisher_->publish(cmd);
}

/**
 * @brief Implementation of execute function for Rotate concrete class of State
 * @param node
 */
void Rotate::execute(WalkerNode *node) {
  float min_range_left = *std::min_element(node->scan_.ranges.begin(),
                                           node->scan_.ranges.begin() + 10);
  float min_range_right = *std::min_element(node->scan_.ranges.end() - 10,
                                            node->scan_.ranges.end());
  if (std::min(min_range_left, min_range_right) > 0.5) {
    node->transition(std::make_shared<MoveForward>());
    node->set_direction(node->get_direction() == Direction::CLOCKWISE
                            ? Direction::COUNTERCLOCKWISE
                            : Direction::CLOCKWISE);
  }

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.0;
  cmd.angular.z =
      (direction_ == Direction::CLOCKWISE ? -node->max_angular_vel_param_
                                          : node->max_angular_vel_param_);
  node->publisher_->publish(cmd);
}

}  // namespace walker

void signalHandler(int sig) {
  RCLCPP_INFO_STREAM(rclcpp::get_logger("WALKER_MAIN"),
                     "Received sigint quitting.");
  rclcpp::shutdown();
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  signal(SIGINT, signalHandler);
  auto node = rclcpp::Node::make_shared("walker_robot");
  auto walker = std::make_shared<walker::WalkerNode>(node);
  rclcpp::Rate loop_rate(10);  // 10 Hz
  while (rclcpp::ok()) {
    walker->run();
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();

  return 0;
}
