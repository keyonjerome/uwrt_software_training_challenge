import launch
import launch_ros.actions
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.events import Shutdown
from launch.substitutions import (EnvironmentVariable, FindExecutable,
                                LaunchConfiguration, LocalSubstitution,
                                PythonExpression)


# RegisterEventHandler(
#             OnProcessStart(
#                 target_action=turtlesim_node,
#                 on_start=[
#                     LogInfo(msg='Turtlesim started, spawning turtle'),
#                     spawn_turtle
#                 ]
#             )
#         ),
def generate_launch_description():

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
    
    clear_turtle_container = ComposableNodeContainer(
            name='clear_turtle_container',
            namespace='ctc',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='software_training_assignment',
                    plugin='composition::clear_turtles',
                    name='clear_turtles',
                    )
            ],
            output='screen',
    )
    spawn_turtle_container = ComposableNodeContainer(
            name='spawn_turtle_container',
            namespace='stc',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='software_training_assignment',
                    plugin='composition::spawn_turtle_nodelet',
                    name='turtle_spawn',
                    ),
            ],
            output='screen',
    )
    turtle_publisher_container = ComposableNodeContainer(
            name='turtle_publisher_container',
            namespace='tpc',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
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
            ],
            output='screen',
    )

    moving_turtle_action_container = ComposableNodeContainer(
            name='moving_turt_action_container',
            namespace='mtac',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='software_training_assignment',
                    plugin='composition::action_turtle',
                    name='action_turtle',
                    ),

            ],
            output='screen',
    )

    on_spawn_turtles =  RegisterEventHandler(
        OnProcessStart(
            target_action=spawn_turtle_container,
            on_start = [turtle_publisher_container,moving_turtle_action_container]
        )
    )


    start_turtlesim = launch_ros.actions.Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='sim',
        output="screen"
        )
    startEvent = RegisterEventHandler(
        OnProcessStart(
            target_action=start_turtlesim,
            on_start = spawn_turtle_container
        )
    )

    return launch.LaunchDescription([startEvent, start_turtlesim,on_spawn_turtles])
