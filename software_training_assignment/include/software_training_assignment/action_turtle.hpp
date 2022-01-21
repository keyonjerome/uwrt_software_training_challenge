#ifndef ACTION_TURTLE_HPP_
#define ACTION_TURTLE_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>                 // ros2 time header
#include <rclcpp_action/rclcpp_action.hpp> // ros2 action header
#include <software_training_assignment/action/software.hpp>
#include <software_training_assignment/visibility.h>

#include <geometry_msgs/msg/twist.hpp> // cmd_vel publisher message
#include <turtlesim/msg/pose.hpp> // header for message to get moving turt position

namespace composition{

class action_turtle : public rclcpp::Node {

public:
  SOFTWARE_TRAINING_PUBLIC
  explicit action_turtle(const rclcpp::NodeOptions &options);

  // nice to have to prevent lengthy repitive code
  using GoalHandleActionServer =
      rclcpp_action::ServerGoalHandle<software_training_assignment::action::Software>;

private:
  // action server
  rclcpp_action::Server<software_training_assignment::action::Software>::SharedPtr
      action_server;

  // publisher to publish moving turtle commands
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;

  // subscriber to get moving turt posiiton
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber;

  // goal callback function
  SOFTWARE_TRAINING_LOCAL
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const software_training_assignment::action::Software::Goal> goal);

  // cancel response callback function
  SOFTWARE_TRAINING_LOCAL
  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleActionServer> goal_handle);

  // handle accepted callback function
  SOFTWARE_TRAINING_LOCAL
  void
  handle_accepted(const std::shared_ptr<GoalHandleActionServer> goal_handle);

  SOFTWARE_TRAINING_LOCAL
  // executioner callback function
  void execute(const std::shared_ptr<GoalHandleActionServer> goal_handle);

  // for subscriber
  float x = 0.0f;
  float y = 0.0f;
  float theta = 0.0f;
  float linear_velocity = 0.0f;
  float angular_velocity = 0.0f;

  static constexpr unsigned int QUEUE{10};
};

}

#endif // ACTION_TURTLE_HPP_
