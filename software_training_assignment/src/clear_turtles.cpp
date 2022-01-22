#include <software_training_assignment/clear_turtles.hpp>

using namespace std::literals::chrono_literals;

namespace composition {

    clear_turtles::clear_turtles(const rclcpp::NodeOptions &options) : Node("clear_turtles",options)  {
        
        // create client
        client = this->create_client<turtlesim::srv::Kill>("/kill");

        // create callback
        // timer = this->create_wall_timer(2s, std::bind(&clear_turtles::kill,this));
        clear_turtles::kill();
    }

    // kill all turtles
    void clear_turtles::kill() {

         // check if service exists
        if (!client->wait_for_service(2s)) {
            if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(),
                        "Interrupted while waiting for service. Exiting!");
            return;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available after waiting");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "TURTLES KILLED.");

        for (std::string &name : turtle_names) {
            auto request = std::make_shared<turtlesim::srv::Kill::Request>();
            request->name = name;

            // create callback to handle response because no 'spin()' is available

            auto callback =
                [this](rclcpp::Client<turtlesim::srv::Kill>::SharedFuture response)
                -> void {
                (void)response;
                RCLCPP_INFO(this->get_logger(), "Turtle  Killed");
                rclcpp::shutdown(); // kill this node
            };

            auto result = client->async_send_request(request, callback);
        }

    }

} // namespace composition

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(composition::clear_turtles)