#ifndef WALKER_H
#define WALKER_H
/**
 * @file walker_node.cpp
 * @author Anirudh Swarankar
 * @brief Souce file for walker node
Copyright [2024] [Anirudh Swarankar]

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
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <vector>
#include <csignal>

namespace walker {

template <typename C> struct is_vector : std::false_type {};
template <typename T, typename A>
struct is_vector<std::vector<T, A>> : std::true_type {};
template <typename C> inline constexpr bool is_vector_v = is_vector<C>::value;

enum class Direction { CLOCKWISE, COUNTERCLOCKWISE };
class WalkerNode;
// state interface
class State {
public:
  virtual void execute(WalkerNode *node) = 0;
  virtual ~State() = default;
};

// concrete class
class IDLE : public State {
public:
  void execute(WalkerNode *node) override;
};

// concrete class
class MoveForward : public State {
public:
  void execute(WalkerNode *node) override;
};

// concrete class
class Rotate : public State {
public:
  Rotate(Direction &direction) : direction_(direction) {}
  void execute(WalkerNode *node) override;

private:
  Direction &direction_;
};

// context

class WalkerNode {

public:
  WalkerNode(rclcpp::Node::SharedPtr node);
  void run();
  sensor_msgs::msg::LaserScan scan_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  void transition(std::shared_ptr<State> state);

  Direction &get_direction();

  bool set_direction(Direction dir);
  double max_linear_vel_param_ = 0.2;
  double max_angular_vel_param_ = 0.2;

private:
  template <typename T>
  void declareAndLoadParameter(
      const std::string &name, T &param, const std::string &description,
      const bool add_to_auto_reconfigurable_params = true,
      const bool is_required = false, const bool read_only = false,
      const std::optional<double> &from_value = std::nullopt,
      const std::optional<double> &to_value = std::nullopt,
      const std::optional<double> &step_value = std::nullopt,
      const std::string &additional_constraints = "");

  rcl_interfaces::msg::SetParametersResult
  parametersCallback(const std::vector<rclcpp::Parameter> &parameters);

  void setup();

  void laserScanCallback(const sensor_msgs::msg::LaserScan &msg);

  void start_callback(const std::shared_ptr<std_srvs::srv::Trigger_Request> req,
                      std::shared_ptr<std_srvs::srv::Trigger_Response> resp);

  void end_callback(const std::shared_ptr<std_srvs::srv::Trigger_Request> req,
                    std::shared_ptr<std_srvs::srv::Trigger_Response> resp);
  std::vector<
      std::tuple<std::string, std::function<void(const rclcpp::Parameter &)>>>
      auto_reconfigurable_params_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      parameters_callback_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr startService_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stopService_;

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<State> state_;
  Direction direction_;
};

} // namespace walker
#endif // WALKER_H