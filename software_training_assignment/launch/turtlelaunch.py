import launch 
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """Generate launch description with multiple components."""

    container = ComposableNodeContainer(
            name='node_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='turtlesim',
                    plugin='turtlesim_node',
                    name="sim"
                ),
                ComposableNode(
                    package='software_training_assignment',
                    plugin='composition::clear_turtles',
                    name='clear_turtles',
                    ),
                ComposableNode(
                    package='software_training_assignment',
                    plugin='composition::spawn_turtle_nodelet',
                    name='turtle_spawn',
                    ),
                ComposableNode(
                    package='software_training_assignment',
                    plugin='composition::turtle_circle_publisher',
                    name='turtle_circle',
                    ),
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])