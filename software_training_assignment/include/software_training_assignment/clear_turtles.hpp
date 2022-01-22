#ifndef CLEAR_TURTLES_HPP_
#define CLEAR_TURTLES_HPP_

#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <software_training_assignment/visibility.h>
#include <turtlesim/srv/kill.hpp>

namespace composition
{

    class clear_turtles : public rclcpp::Node
    {
    public:
        SOFTWARE_TRAINING_PUBLIC        
        explicit clear_turtles(const rclcpp::NodeOptions &options);

    private:
        rclcpp::Client<turtlesim::srv::Kill>::SharedPtr client;
        rclcpp::TimerBase::SharedPtr timer;

        // all the turtles
        std::vector<std::string> turtle_names = {"moving_turtle",
                                                 "stationary_turtle"};

        // one turtle
        // std::vector<std::string> turtle_names = {"turtle1"};

        SOFTWARE_TRAINING_LOCAL
        void kill();
    };

} // namespace composition
#endif // CLEAR_TURTLES_HPP_