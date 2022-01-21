# UWRT Software  Training Assignment - Complete

## How To Run
1. Clone this repository.
2. Source your ROS2 installation. `source /opt/ros/foxy/setup.bash`.
3. `cd` into this repository.
4. Run `colcon build` in this repository.
5. Open a new terminal.
6. `cd` into this repository in the new terminal.
7. Run `. install/setup.bash` in the new terminal.
8. Run `ros2 launch software_training_assignment testlaunch.py` in the new terminal. This will launch all nodes.
9. Open a new terminal.
10. In the new, third terminal, `cd` into this repository.
11. Run `. install/setup.bash` in this third terminal.
12. Send a goal to the action server node using `ros2 action send_goal /action_turtle software_training_assignment/action/Software "{x_pos: 5 , y_pos: 5}"`.
