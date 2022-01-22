

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
            // RCLCPP_INFO(this->get_logger(),"UPDATED X:%f || UPDATED Y:%f",this->action_turtle::x,this->action_turtle::y);
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
        RCLCPP_INFO(this->get_logger(), "linear X:%f Y:%f", goal->x_pos,
                    goal->y_pos);
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
        

        const float time_constant = 1.250f;
        // get goal data for later
        const auto goal = goal_handle->get_goal();

        // create result
        std::unique_ptr<software_training_assignment::action::Software::Result> result =
            std::make_unique<software_training_assignment::action::Software::Result>();


        auto goal_x = goal->x_pos;
        auto goal_y = goal->y_pos;

        // create reference to current information for ease of use
        float &curr_x = this->action_turtle::x;
        float &curr_y = this->action_turtle::y;
        // float &curr_theta = this->action_turtle::theta;



        float TOL = 0.25;

        // Have you heard of our lord and saviour, v = d/t ?
        float vel_x = (goal_x - curr_x)/time_constant;
        float vel_y = (goal_y - curr_y)/time_constant;

        RCLCPP_INFO(this->get_logger(),"VEL_X:%f, VEL_Y:%f", vel_x, vel_y);


        RCLCPP_INFO(this->get_logger(),"CURR_X:%f, CURR_Y:%f || GOAL_X: %f, GOAL_Y: %f",curr_x,curr_y,goal_x,goal_y);
        while(rclcpp::ok() && ( 
            !(curr_x >= goal_x - TOL && curr_x <= goal_x + TOL) || !(curr_y >= goal_y - TOL && curr_y <= goal_y + TOL))  
        ) {
            auto message = std::make_unique<geometry_msgs::msg::Twist>();
            message->linear.x = vel_x;
            message->linear.y = vel_y;
            message->linear.z = 0.0f;
            message->angular.x = 0.0f;
            message->angular.y = 0.0f;
            message->angular.z = 0.0f;

            // create feedback
            std::unique_ptr<software_training_assignment::action::Software::Feedback> feedback =
            std::make_unique<software_training_assignment::action::Software::Feedback>();

            feedback->dist = sqrt(pow(goal->x_pos - curr_x,2)+ pow(goal->y_pos - curr_y,2));

            this->publisher->publish(std::move(message));

            goal_handle->publish_feedback(std::move(feedback));

            RCLCPP_INFO(this->get_logger(),"CURR_X:%f, CURR_Y:%f || GOAL_X: %f, GOAL_Y: %f",curr_x,curr_y,goal_x,goal_y);
            RCLCPP_INFO(this->get_logger(),"CMD_VEL_X:%f, CMD_VEL_Y:%f",vel_x,vel_y);

            // loop_rate.sleep();
        }
       
         // if goal is done
        if (rclcpp::ok()) {

            auto message = std::make_unique<geometry_msgs::msg::Twist>();
            message->linear.x = 0.0f;
            message->linear.y = 0.0f;
            message->linear.z = 0.0f;
            message->angular.x = 0.0f;
            message->angular.y = 0.0f;
            message->angular.z = 0.0f;
            this->publisher->publish(std::move(message));


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