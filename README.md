# kinova_arm_apps

# Instructions

## Dependencies

```
sudo apt install ros-kinetic-timed-roslaunch
```

## For simulation
```
roslaunch kortex_gazebo spawn_kortex_robot.launch

rosrun moveit_commander moveit_commander_cmdline.py __ns:=my_gen3
> use arm
> go home
> go vertical
```

## For real robot
```
roslaunch kortex_driver kortex_driver.launch
```
To get current joint position in degrees
```
rosservice call /my_gen3/base/get_measured_joint_angles "input: {}"
```
To get current cartesian position
```
rosservice call /my_gen3/base/get_measured_cartesian_position "input: {}"
```

## For pick and place
Connect arm with lan cable (will not work via wifi connection)
```
roslaunch kinova_arm_apps kinova_arm.launch
roslaunch kinova_arm_apps closest_obj.launch

roslaunch kinova_arm_apps pick_place.launch

rostopic pub /pcl_closest_obj/event_in geometry_msgs/String e_start
```
Check the output of `pick_place.launch`, if it says invalid pose, send `e_start`
again. And, finally
```
rostopic pub /pick_and_place/event_in geometry_msgs/String e_start
```

## Using MoveIt! client
Connect either with LAN cable or WLAN.

*Note*: `kortex_driver.launch` or custom launch invoking `kortex_driver.launch` should initialise MoveIt! server. Please check `kortex_driver.launch`

```
<arg name="start_moveit" default="true"/>
```

Alternatively, you can pass `start_moveit` as an argument in custom launch file, in case you don't want to start moveit everytime.

Next, start the `moveit_commander`: 
```
rosrun moveit_commander moveit_commander_cmdline.py __ns:=my_gen3
> use arm (or gripper)
> go home
> go vertical
```

The robot's srdf can be found in `kortex_move_it_config` package on your local machine. The srdf to be updated is located in:
```
roscd kortex_move_it_config && cd ../gen3_robotiq_2f_85_move_it_config/config/
```

*Final note*: To use MoveIt! client, ensure that the `kortex_driver` has the correct argument set, then ensure the namespace of the robot is passed to your MoveIt! client script.

