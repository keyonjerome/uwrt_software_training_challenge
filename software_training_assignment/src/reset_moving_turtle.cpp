#include <chrono>
#include <software_training_assignment/reset_moving_turtle.hpp>

using namespace std::literals::chrono_literals;

using namespace std::placeholders;

namespace composition
{

    reset_moving_turtle::reset_moving_turtle(const rclcpp::NodeOptions &options) : Node("reset_moving_turtle", options)
    {
        
        // I believe this only creates the service, but does not call it. 

        // create client
        // teleport_absolute may need to be TeleportAbsolute
        this->client = this->create_client<turtlesim::srv::TeleportAbsolute>("/moving_turtle/teleport_absolute");

        // // create callback
        // timer = this->create_wall_timer(2s, std::bind(&reset_moving_turtle::reset_moving_turtle,this));

        // create service
        this->service = this->create_service<software_training_assignment::srv::ResetMovingTurtle>("/reset_moving_turtle", std::bind(&reset_moving_turtle::service_callback, this, _1, _2));
    }

    void reset_moving_turtle::service_callback(const std::shared_ptr<software_training_assignment::srv::ResetMovingTurtle::Request> request,
                                               std::shared_ptr<software_training_assignment::srv::ResetMovingTurtle::Response> response)
    {

        (void)request; // request is not needed

        RCLCPP_INFO(this->get_logger(), "Starting reset_moving_turtle service...");

        // wait until service is available
        if (!client->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(),
                             "Interrupted while waiting for service. Exiting!");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available after waiting");
            return;
        }

        auto client_request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
        client_request->x = reset_moving_turtle::x_coord;
        client_request->y = reset_moving_turtle::y_coord;
        client_request->theta = reset_moving_turtle::theta_coord;

        // create callback to handle response because no 'spin()' is available

        auto response_callback =
            [this](rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedFuture teleport_response)
            -> void
        {
            (void)teleport_response;
            RCLCPP_INFO(this->get_logger(), "Moving turtle reset completed.");
            // rclcpp::shutdown(); // need this or else will keep on executing callback -
            //                     // only want to execute once!
        };

        auto result = client->async_send_request(client_request, response_callback);

        response->success = true;
    }

} // namespace composition

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(composition::reset_moving_turtle)