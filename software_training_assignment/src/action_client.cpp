#include <memory>
// #include <software_training_assignment/spawn_turtle_nodelet.hpp>
#include <software_training_assignment/action_client.hpp>
#include <software_training_assignment/action/software.hpp>
using namespace std::placeholders;
using namespace std::literals::chrono_literals;

namespace composition
{

    action_client::action_client(const rclcpp::NodeOptions &options)
        : Node("action_client", options)
    {

        this->client = this->create_client<software_training_assignment::action::Software>(this->get_node_base_interface(),
                                                                  this->get_node_graph_interface(),
                                                                  this->get_node_logging_interface(),
                                                                  this->get_node_waitables_interface(), "/action_client");
        this->timer = this->create_wall_timer(
            std::chrono::milliseconds(5000),
            std::bind(&action_client::send_goal, this));
    };

    void action_client::send_goal()
    {

        using namespace std::placeholders;

        this->timer->cancel();

        if (!this->client->wait_for_action_server(std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
         
            return;
        }

        auto goal_msg = software_training_assignment::action::Software::Goal();

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<software_training_assignment::action::Software>::SendGoalOptions();

        send_goal_options.goal_response_callback =
            std::bind(&action_client::goal_response_callback, this, _1);

        send_goal_options.feedback_callback =
            std::bind(&action_client::feedback_callback, this, _1, _2);

        send_goal_options.result_callback =
            std::bind(&action_client::result_callback, this, _1);

        this->client->async_send_goal(goal_msg, send_goal_options);
    }

    void action_client::goal_response_callback(GoalHandleActionClient::SharedPtr goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void action_client::feedback_callback(
        GoalHandleActionClient::SharedPtr,
        const std::shared_ptr<const software_training_assignment::action::Software::Feedback> feedback)
    {
        std::stringstream ss;
        ss << "Next number in sequence received: ";
        for (auto number : feedback->partial_sequence)
        {
            ss << number << " ";
        }
        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    }

    void action_client::result_callback(const GoalHandleActionClient::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
        }
        std::stringstream ss;
        ss << "Result received: ";
        for (auto number : result.result->sequence)
        {
            ss << number << " ";
        }
        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
        rclcpp::shutdown();
    }
}

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(composition::action_client)
