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





    continuous_node_container = ComposableNodeContainer(
            name='node_container',
            namespace='',
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
                ComposableNode(
                    package='software_training_assignment',
                    plugin='composition::reset_moving_turtle',
                    name='reset_moving_turtle',
                    ),
                ComposableNode(
                    package='software_training_assignment',
                    plugin='composition::action_turtle',
                    name='action_turtle',
                    ),    
            ],
            output='screen',
    )
    start_turtlesim = launch_ros.actions.Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='sim',
        output="screen"
        )


    action_location = " software_training_assignment/action/Software"
    action_name = " /action_turtle"
    # ros2 action send_goal /action_turtle software_training_assignment/action/Software "{x_pos: 5 , y_pos: 5}
    call_action = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' action send_goal ',
            action_name,
            action_location,
            ' " {x_pos: 5 , y_pos: 5}"'
        ]],
        shell=True
    )
    # ros2 service call /reset_moving_turtle software_training_assignment/srv/ResetMovingTurtle
    service_name = " /reset_moving_turtle"
    service_location = " software_training_assignment/srv/ResetMovingTurtle"
    call_service = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' service call',
            service_name,
            service_location
        ]],
        shell=True
    )


    on_turtlesim_start = RegisterEventHandler(
        OnProcessStart(
            target_action=start_turtlesim,
            on_start = clear_turtle_container
        )
    )
    on_clear_complete = RegisterEventHandler(
            OnExecutionComplete(
                target_action=clear_turtle_container,
                on_completion=spawn_turtle_container
            )
        )
    on_spawn_complete = RegisterEventHandler(
            OnExecutionComplete(
                target_action=spawn_turtle_container,
                on_completion=[continuous_node_container,TimerAction(period=3.0,actions=[call_action]),TimerAction(period=8.0,actions=[call_service])]
            )
        )

    '''
    1. Launch turtlesim node.
    2. On turtlesim node process start, start clear turtle node
    3. On clear turtlenode execution complete, start spawn turtle node
    4. On spawn turtle node execution complete, start up all other nodes simultaneously
    5. After 3 seconds (using TimerAction), send action to move moving turtle to point
    6. After 8 seconds, reset moving turtle with service call 
    '''

    return launch.LaunchDescription([start_turtlesim,on_turtlesim_start,on_clear_complete,on_spawn_complete])
