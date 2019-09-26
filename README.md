# kinova_arm_apps

# Instructions

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
To get current position
```
rosservice call /my_gen3/base/get_measured_joint_angles "input: {}"
```

## For pick and place
Connect arm with lan cable (will not work via wifi connection)
```
roslaunch kinova_arm_apps kinova_arm.launch
roslaunch kinova_arm_apps closest_obj.launch

rosservice call /my_gen3/base/activate_publishing_to_action_topic TAB_COMPLETE

roslaunch kinova_arm_apps pick_place.launch

rostopic pub /pcl_closest_obj/event_in e_start
```
Check the output of `pick_place.launch`, if it says invalid pose, send `e_start`
again. And, finally
```
rostopic pub /pick_and_place/event_in e_start
```



