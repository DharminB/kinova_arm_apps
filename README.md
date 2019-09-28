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



