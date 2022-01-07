#include <memory>
#include <software_training_assignment/turtle_circle_publisher.hpp>

using namespace std::literals::chrono_literals;

namespace composition {

turtle_circle_publisher::turtle_circle_publisher(const rclcpp::NodeOptions &options)
    : Node("turtle_circle_publisher") {

    
  auto publisher_callback = [this](void) -> void {
    auto message = std::make_unique<geometry_msgs::msg::Twist>();
    message->linear.x = turtle_circle_publisher::coordinates::linear::x;
    message->linear.y = turtle_circle_publisher::coordinates::linear::y;
    message->linear.z = turtle_circle_publisher::coordinates::linear::z;

    message->angular.x = turtle_circle_publisher::coordinates::angular::x;
    message->angular.y = turtle_circle_publisher::coordinates::angular::y;
    message->angular.z = turtle_circle_publisher::coordinates::angular::z;

    this->publisher->publish(std::move(message));
  };

  // create publisher
  // create a Quality of Service object with a history/ depth of 10 calls
  this->publisher = this->create_publisher<geometry_msgs::msg::Twist>(
      "/turtle1/cmd_vel", rclcpp::QoS(QUEUE));


  // publish values every 1ms
  this->timer = this->create_wall_timer(1ms, publisher_callback);
    };

 
}

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(composition::turtle_circle_publisher)
