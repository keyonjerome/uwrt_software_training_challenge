
#ifndef TURTLE_CIRCLE_PUBLISHER_HPP_
#define TURTLE_CIRCLE_PUBLISHER_HPP_

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <software_training_assignment/visibility.h>

namespace composition {

class turtle_circle_publisher : public rclcpp::Node {

public:
  SOFTWARE_TRAINING_PUBLIC
  explicit turtle_circle_publisher(const rclcpp::NodeOptions &options);

private:
  // publisher
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;

  // callback timer
  rclcpp::TimerBase::SharedPtr timer;

  // set quality of service depth - AKA a backlog
  static constexpr unsigned int QUEUE{10};

  // namespace for values
  typedef struct point {

    typedef struct linear {
      static constexpr float x = 2;
      static constexpr float y = 0;
      static constexpr float z = 0;
    } linear;

    typedef struct angular {
      static constexpr float x = 0;
      static constexpr float y = 0;
      static constexpr float z = 1;

    } angular;

  } coordinates;
};

} // namespace composition

#endif //  TURTLE_CIRCLE_PUBLISHER_HPP_