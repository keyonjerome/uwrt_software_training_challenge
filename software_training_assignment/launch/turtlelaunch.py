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
                ComposableNode(
                    package='software_training_assignment',
                    plugin='composition::turtle_publisher',
                    name='turtle_publisher',
                    ),
                ComposableNode(
                    package='software_training_assignment',
                    plugin='composition::reset_moving_turtle',
                    name='reset_moving_turtle',
                    ),        
            ],
            output='screen',
    )
    turtlesimlaunch = ComposableNodeContainer(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='turtle1',
            composable_node_descriptions=[]
    )
    

    return launch.LaunchDescription([container,turtlesimlaunch])