#ifndef RESET_MOVING_TURTLE_HPP_
#define RESET_MOVING_TURTLE_HPP_

#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

// #include <software_training_assignment/visibility.h>
#include <software_training_assignment/visibility.h>
#include <turtlesim/srv/teleport_absolute.hpp>

namespace composition
{

    class reset_moving_turtle : public rclcpp::Node
    {
    public:
        SOFTWARE_TRAINING_PUBLIC
        explicit reset_moving_turtle(const rclcpp::NodeOptions &options);

    private:
        rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr client;
        rclcpp::TimerBase::SharedPtr timer;
        // create service that will reset turtle to starting point
        rclcpp::Service<software_training_assignment::srv::ResetMovingTurtle>::SharedPtr service;


        std::string turtle_to_move = "moving_turtle";
        float x_coord = 0.0F;
        float y_coord = 0.0F;
        float theta_coord = 0.0F;

        SOFTWARE_TRAINING_LOCAL
        void service_callback(
            const std::shared_ptr<software_training_assignment::srv::ResetMovingTurtle::Request> request,
             std::shared_ptr<software_training_assignment::srv::ResetMovingTurtle::Response> response);
    };

} // namespace composition
#endif // RESET_MOVING_TURTLE_HPP_