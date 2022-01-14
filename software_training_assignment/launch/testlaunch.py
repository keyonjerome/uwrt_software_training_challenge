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
    startcontainer = " run rclcpp_components node_container"
    startComponent = " component load /node_container spawn_turtles composition::spawn_turtle_nodelet -p writer_name:='composition::spawn_turtle_nodelet'"
    spawn_turtles = ExecuteProcess(
        cmd=[[FindExecutable(name='ros2'), startcontainer],[
            FindExecutable(name='ros2'), startComponent
        ]],
        shell=True
    )

    # spawn_turtles = launch_ros.actions.Node(
    #         package="software_training_assignment",
    #         executable="turtle_spawn",
    #         name="turtle_spawn",
    #         output="screen"),

    start_turtlesim = launch_ros.actions.Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='sim',
        output="screen"
        )
    # turtlesim_node = Node(
    #     package='turtlesim',
    #     executable='turtlesim_node',
    #     name='sim'
    # )

    startEvent = RegisterEventHandler(
        OnProcessStart(
            target_action=start_turtlesim,
            # on_start=[
            #     LogInfo(msg='Turtlesim started, spawning turtle'),
            #     spawn_turtles
            # ]
            # on_start = launch_ros.actions.load_composable_nodes.LoadComposableNodes(container)
            on_start = container
        )
    )

    return launch.LaunchDescription([startEvent, start_turtlesim])
