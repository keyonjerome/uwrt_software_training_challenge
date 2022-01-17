

#include <cmath>
#include <memory>

#include <software_training_assignment/action_turtle.hpp>

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace composition
{
    // on node creation, create publisher, action server, turtle pose subscriber...
    action_turtle::action_turtle(const rclcpp::NodeOptions &options) : Node("action_turtle", options)
    {

        this->action_server = rclcpp_action::create_server<software_training_assignment::action::Software>(this, "action_turtle",
                                                                                                             std::bind(&action_turtle::handle_goal, this, _1, _2),
                                                                                                             std::bind(&action_turtle::handle_cancel, this, _1),
                                                                                                             std::bind(&action_turtle::handle_accepted, this, _1));

        this->publisher = this->create_publisher<geometry_msgs::msg::Twist>(
            "/moving_turtle/cmd_vel", rclcpp::QoS(QUEUE));

        auto subscriber_callback =
            [this](const turtlesim::msg::Pose::SharedPtr msg) -> void
        {
            this->action_turtle::x = msg->x;
            this->action_turtle::y = msg->y;
            this->action_turtle::theta = msg->theta;
            this->action_turtle::linear_velocity = msg->linear_velocity;
            this->action_turtle::angular_velocity = msg->angular_velocity;
        };

        // create subscriber
        this->subscriber = this->create_subscription<turtlesim::msg::Pose>(
            "/moving_turtle/pose", QUEUE, subscriber_callback);
    }  

    rclcpp_action::CancelResponse action_turtle::handle_cancel(
    const std::shared_ptr<GoalHandleActionServer> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
  }

    // respond to goal request
    rclcpp_action::GoalResponse action_turtle::handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const software_training_assignment::action::Software::Goal> goal)
    {

        (void)uuid; // not needed - prevents compiler warnings/errors -Wall flag
        RCLCPP_INFO(this->get_logger(), "Goal Received");
        RCLCPP_INFO(this->get_logger(), "linear X:%f Y:%f Z:%f", goal->linear_pos.x,
                    goal->linear_pos.y, goal->linear_pos.z);
        RCLCPP_INFO(this->get_logger(), "angular X:%f Y:%f Z:%f", goal->angular_pos.x,
                    goal->angular_pos.y, goal->angular_pos.z);

        // accept zee goal me friendo. COMPLETE ZEE GOAL!
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    void action_turtle::handle_accepted(const std::shared_ptr<GoalHandleActionServer> goal_handle)
    {
        // using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&action_turtle::execute, this, _1), goal_handle}.detach();
    }

    /*
    # Feedback /moving_turtle/pose
    float32 x_pos 
    float32 y_pos
    float32 theta_pos
    */

    // executioner callback function
    void action_turtle::execute(const std::shared_ptr<GoalHandleActionServer> goal_handle)
    {

        RCLCPP_INFO(this->get_logger(), "Executing goal");
        rclcpp::Rate loop_rate(1);
        

        rclcpp::Time start_time = this->now(); // get the current time

        // get goal data for later
        const auto goal = goal_handle->get_goal();

        // create feedback
        std::unique_ptr<software_training_assignment::action::Software::Feedback> feedback =
            std::make_unique<software_training_assignment::action::Software::Feedback>();


        // create result
        std::unique_ptr<software_training_assignment::action::Software::Result> result =
            std::make_unique<software_training_assignment::action::Software::Result>();


        // feedback->dist = sqrt((goal->linear_pos_x - curr_x)^2 + (goal->linear_pos_y - curr_y)^2);
        

        // create reference to feedback for ease of use
        float &curr_x = feedback->x_pos;
        float &curr_y = feedback->y_pos;
        float &curr_theta = feedback->theta_pos;

        // keep track of linear feedback
        float lin_x = 0.0f;
        float lin_y = 0.0f;
        float lin_z = 0.0f;

        // keep track of angular feedback
        float ang_x = 0.0f;
        float ang_y = 0.0f;
        float ang_z = 0.0f;

          // heavy lifting
        while (rclcpp::ok() &&
                (lin_x < goal->linear_pos.x || lin_y < goal->linear_pos.y ||
                lin_z < goal->linear_pos.z || ang_x < goal->angular_pos.x ||
                ang_y < goal->angular_pos.y || ang_z < goal->angular_pos.z)) {

            // check if goal has been canceled
            if (goal_handle->is_canceling()) {
                RCLCPP_INFO(this->get_logger(), "Goal Canceled");

                // get the time it has taken thus far and update result
                rclcpp::Time curr_time = this->now();
                rclcpp::Duration time = curr_time - start_time;
                long int duration{time.nanoseconds()};
                result->duration = duration;

                goal_handle->canceled(std::move(result));
                return;
            }
        

        
            auto message = std::make_unique<geometry_msgs::msg::Twist>();

            message->linear.x = (lin_x < goal->linear_pos.x) ? lin_x++ : lin_x;
            message->linear.y = (lin_y < goal->linear_pos.y) ? lin_y++ : lin_y;
            message->linear.z = (lin_z < goal->linear_pos.z) ? lin_z++ : lin_z;

            message->angular.x =   (ang_x < goal->angular_pos.x) ? ang_x++ : ang_x;
            message->angular.y =   (ang_y < goal->angular_pos.y) ? ang_y++ : ang_y;
            message->angular.z = (ang_z < goal->angular_pos.z) ? ang_z++ : ang_z;

            this->publisher->publish(std::move(message));

            // now compute feedback
            curr_x = this->action_turtle::x - lin_x;
            curr_y = this->action_turtle::y - lin_y;

            // Orson's theta math
            float theta{0};

            // scope this stuff cause we dont need it afterwards
            {

                float x1{lin_x}, x2{lin_y}, x3{lin_z};

                float magnitude{static_cast<float>(sqrt((x1 * x1) + (x2 * x2) + (x3 * x3)))};

                theta = acos(x3 / magnitude);
            }

            curr_theta = this->action_turtle::theta - theta;

            // publish feedback
            goal_handle->publish_feedback(std::move(feedback));

            loop_rate.sleep(); // control the rate at which the loop, loops through

        }

         // if goal is done
        if (rclcpp::ok()) {

            rclcpp::Time end = this->now();               // get end time
            rclcpp::Duration duration = end - start_time; // compute time taken
            long int res_time{duration.nanoseconds()};    // should be uint64_t

            // fill in result
            result->duration = res_time;

            // set the result
            goal_handle->succeed(
                std::move(result)); // move ownership so ptr is still usable
            RCLCPP_INFO(this->get_logger(), "Finish Executing Goal");
        }


    }

}

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(composition::action_turtle)