#ifndef ACTION_CLIENT_HPP_
#define ACTION_CLIENT_HPP_

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
#include <turtlesim/msg/pose.hpp>      // header for message to get moving turt position

namespace composition
{

    class action_client : public rclcpp::Node
    {

    public:
        SOFTWARE_TRAINING_PUBLIC
        explicit action_client(const rclcpp::NodeOptions &options);

        // nice to have to prevent lengthy repitive code
        using GoalHandleActionClient =
            rclcpp_action::ClientGoalHandle<software_training_assignment::action::Software>;
        
        // using SoftwareAction = software_training_assignment::action::Software;

        SOFTWARE_TRAINING_PUBLIC
        void send_goal();
    private:
        // action client
        rclcpp_action::Client<software_training_assignment::action::Software>::SharedPtr
            client;

        rclcpp::TimerBase::SharedPtr timer;

        // goal callback function
        SOFTWARE_TRAINING_LOCAL
        void goal_response_callback(GoalHandleActionClient::SharedPtr goal_handle);

        // cancel response callback function
        SOFTWARE_TRAINING_LOCAL
        void
        feedback_callback(GoalHandleActionClient::SharedPtr, const std::shared_ptr<const SoftwareAction::Feedback> feedback);

        // handle accepted callback function
        SOFTWARE_TRAINING_LOCAL
        void
        result_callback(const GoalHandleActionClient::WrappedResult & result);

    };

}

#endif // ACTION_CLIENT_HPP_
