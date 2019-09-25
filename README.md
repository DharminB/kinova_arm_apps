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
