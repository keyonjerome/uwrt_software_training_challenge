#ifndef CLEAR_TURTLES_HPP_
#define CLEAR_TURTLES_HPP_

#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

// #include <software_training/visibility.h>
#include <turtlesim/srv/kill.hpp>

namespace composition
{

    class clear_turtles : public rclcpp::Node
    {
    public:
        explicit clear_turtles(const rclcpp::NodeOptions &options);

    private:
        rclcpp::Client<turtlesim::srv::Kill>::SharedPtr client;
        rclcpp::TimerBase::SharedPtr timer;

        // all the turtles
        std::vector<std::string> turtle_names = {"turtle1", "moving_turtle",
                                                 "stationary_turtle"};

        //   SOFTWARE_TRAINING_LOCAL
        void kill();
    };

} // namespace composition
#endif // CLEAR_TURTLES_HPP_