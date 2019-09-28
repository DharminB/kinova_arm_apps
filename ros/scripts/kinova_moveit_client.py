#!/usr/bin/env python
import sys
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_srvs.srv import Empty

class KinovaMoveItClient(object):
    '''
        MoveIt! wrapper for kinova gen3 arm.
    '''
    def __init__(self):

        # Initialize the node
        super(KinovaMoveItClient, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('kinova_moveit_client')

        rospy.loginfo("Initializing node " + rospy.get_name() + " in namespace " + rospy.get_namespace())

        self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)

        # Create the MoveItInterface necessary objects
        arm_group_name = "arm"
        self.robot = moveit_commander.RobotCommander("robot_description")
        self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
        self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
        self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                                                            moveit_msgs.msg.DisplayTrajectory,
                                                            queue_size=20)

        if self.is_gripper_present:
            gripper_group_name = "gripper"
            self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())

        rospy.loginfo("Initialized node " + rospy.get_name() + " in namespace " + rospy.get_namespace())

    def reach_named_position(self, group_name, target):

        if group_name=="arm":
            group = self.arm_group
        elif group_name=="gripper":
            group = self.gripper_group
        else:
            rospy.logerror("[ERROR] Given group name does not exist. Available groups: arm, gripper")
            sys.exit(1)

        # Going to one of those targets
        rospy.loginfo("Going to named target " + target)
        # Set the target
        group.set_named_target(target)
        # Plan the trajectory
        planned_path1 = group.plan()
        # Execute the trajectory and block while it's not finished
        group.execute(planned_path1, wait=True)

    def reach_joint_angles(self, tolerance):
        arm_group = self.arm_group

        # Get the current joint positions
        joint_positions = arm_group.get_current_joint_values()
        rospy.loginfo("Printing current joint positions before movement :")
        for p in joint_positions: rospy.loginfo(p)

        # Set the goal joint tolerance
        self.arm_group.set_goal_joint_tolerance(tolerance)

        # Set the joint target configuration
        joint_positions[0] = pi/2
        joint_positions[1] = 0
        joint_positions[2] = pi/4
        joint_positions[3] = -pi/4
        joint_positions[4] = 0
        joint_positions[5] = pi/2
        joint_positions[6] = 0.2
        arm_group.set_joint_value_target(joint_positions)

        # Plan and execute in one command
        arm_group.go(wait=True)

        # Show joint positions after movement
        new_joint_positions = arm_group.get_current_joint_values()
        rospy.loginfo("Printing current joint positions after movement :")
        for p in new_joint_positions: rospy.loginfo(p)

    def get_cartesian_pose(self):
        arm_group = self.arm_group

        # Get the current pose and display it
        pose = arm_group.get_current_pose()
        rospy.loginfo("Actual cartesian pose is : ")
        rospy.loginfo(pose.pose)

        return pose.pose

    def reach_cartesian_pose(self, pose, tolerance, constraints):
        arm_group = self.arm_group

        # Set the tolerance
        arm_group.set_goal_position_tolerance(tolerance)

        # Set the trajectory constraint if one is specified
        if constraints is not None:
            #import pdb; pdb.set_trace()
            arm_group.set_path_constraints(constraints)

        # Get the current Cartesian Position
        arm_group.set_pose_target(pose)

        # Plan and execute
        rospy.loginfo("Planning and going to the Cartesian Pose")
        arm_group.go(wait=True)

    def reach_gripper_position(self, relative_position):
        gripper_group = self.gripper_group

        # We only have to move this joint because all others are mimic!
        gripper_joint = self.robot.get_joint("gripper_finger1_joint")
        gripper_max_absolute_pos = gripper_joint.max_bound()
        gripper_joint.move(relative_position * gripper_max_absolute_pos, True)

def main():
    '''
        Execute simple commands for the purposes of a demo.
    '''
    test = KinovaMoveItClient()

    rospy.loginfo("Press any key to go to named target 'vertical'")
    raw_input()
    test.reach_named_position(group_name='arm', target="vertical")

    rospy.loginfo("Press any key to go to named target 'home'")
    raw_input()
    test.reach_named_position(group_name='arm', target="home")

    rospy.loginfo("Press any key to go to named target 'retract'")
    raw_input()
    test.reach_named_position(group_name='arm', target="retract")

    rospy.loginfo("Press any key to go to named target 'demo_pose'")
    raw_input()
    test.reach_named_position(group_name='arm', target="demo_pose")

    rospy.loginfo("Press any key to wave hello!")
    raw_input()

    for _ in range(3):

        rospy.loginfo("Waving left! :)")
        test.reach_named_position(group_name='arm', target="wave_left")
        test.reach_named_position(group_name='gripper', target="closed")
        test.reach_named_position(group_name='gripper', target="opened")

        rospy.loginfo("Waving right! :)")
        test.reach_named_position(group_name='arm', target="wave_right")
        test.reach_named_position(group_name='gripper', target="closed")
        test.reach_named_position(group_name='gripper', target="opened")
      
if __name__ == '__main__':
    main()
