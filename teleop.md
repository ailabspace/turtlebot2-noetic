# Teleop the Turtlebot2

Perhaps the easiest and yet fun thing to do with a mobile robot is to be able to tele-operate it. If you have set up ROS multimachine, you can teleop the mobile robot while viewing its camera giving you a basic telepresence.
- The most conveniently available teleop controller is probably the PC keyboard. The keyboard may be attached (ideally wireless) to the Turtlebot PC, or a remote PC (in case of ROS multimachine setup).
- Another conveniently available controllers are the game joysticks, e.g. the Sony Playstation DualShock controllers.
- Here we setup for keyboard, PS3 and PS4 controllers.

## Teleop with keyboard

The robot can be conveniently operated if you have a wireless keyboard, or you work on a remote PC on ROS multimachines environment.
- In the case of ROS multimachines environment, nodes that access a specific hardware (e.g. Kobuki, Astra camera, keyboard) should be running on the PC where the hardware is attached to. 

### Telepresence

- Run each command in a new terminal

 - Bringup (in case of ROS multimachines environment, run on the PC where the Kobuki is attached, i.e. the Turtlebot2)
``` bash
roslaunch turtlebot_bringup minimal.launch
```
 - Use `rqt_image_view` to select the image topic to view

``` bash
rosrun rqt_image_view rqt_image_view
```

 - Teleop (in case of ROS multimachines environment, run on the PC where the keyboard is attached, e.g. the remote PC)
``` bash
roslaunch turtlebot_teleop keyboard_teleop.launch
```

### Mapping

- Run each command in a new terminal

 - Bringup (in case of ROS multimachines environment, run on the PC where the Kobuki is attached, i.e. the Turtlebot2)
``` bash
roslaunch turtlebot_bringup minimal.launch
```

 - Teleop (in case of ROS multimachines environment, run on the PC where the keyboard is attached, e.g. the remote PC)
``` bash
roslaunch turtlebot_teleop keyboard_teleop.launch
```

 - SLAM (in case of ROS multimachines environment, run on the PC where the Astra camera is attached, i.e. the Turtlebot2; this launch file starts the Astra node)
``` bash
roslaunch turtlebot_navigation gmapping_demo.launch
```

 - Visualization (in case of ROS multimachines environment, this can be run on a more powerful remote PC where you have a display)
   - Use teleop to move the robot to see the SLAM in action
``` bash
roslaunch turtlebot_rviz_launchers view_navigation.launch
```

## Teleop with joystick

