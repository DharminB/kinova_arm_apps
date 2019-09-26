#!/usr/bin/env python
from __future__ import print_function

import rospy
import copy
import math
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from kinova_arm_apps.full_arm_movement import FullArmMovement

class PickAndPlace(object):

    """pick and place code using full arm movement and 3d segmentation perception"""

    def __init__(self):
        self.fam = FullArmMovement()
        default_boundary_safety = {"x_min": -0.1, "x_max": 0.1, "y_min": -0.45,\
        		           "y_max": -0.25, "z_min": 0.0, "z_max": 0.1}
	self.boundary_safety = rospy.get_param("~boundary_safety ", default_boundary_safety )
	self.joint_angles = rospy.get_param("~joint_angles", {})
	print (self.boundary_safety)

        # Subscribers
        self.perception_pose_sub = rospy.Subscriber('~pose_in', PoseStamped, self.perception_pose_cb)
        self.event_in_sub = rospy.Subscriber('~event_in', String, self.event_in_cb)
        self.debug_pose_pub = rospy.Publisher('~debug_pose', PoseStamped, queue_size=1)

        self.setup_arm_for_pick()
	rospy.loginfo("READY!")

    def perception_pose_cb(self, msg):
        rospy.loginfo(msg.pose.position)
        if self.boundary_safety["x_min"] < msg.pose.position.x < self.boundary_safety["x_max"] and \
                self.boundary_safety["y_min"] < msg.pose.position.y < self.boundary_safety["y_max"] and \
                self.boundary_safety["z_min"] < msg.pose.position.z < self.boundary_safety["z_max"]:
            self.perception_pose = msg
        else:
            rospy.logerr("Input pose out of bound")

    def event_in_cb(self, msg):
        if msg.data == 'e_start':
            if self.perception_pose is None:
                return
            print(self.fam.current_ee_pose)
            debug_pose = copy.deepcopy(self.perception_pose)
            debug_pose.pose.position.z = self.fam.current_ee_pose[2] - 0.03
            debug_pose.pose.orientation = FullArmMovement.quat_from_rpy(*map(math.radians, self.fam.current_ee_pose[3:]))
            # debug_pose = self.fam.get_pose_from_current_ee_pose()
            # debug_pose.header.frame_id = 'base_link'
            # print(debug_pose)
            self.debug_pose_pub.publish(debug_pose)
            self.fam.send_cartesian_pose(debug_pose)
            debug_pose.pose.position.z = self.perception_pose.pose.position.z + 0.03
            self.debug_pose_pub.publish(debug_pose)
            self.fam.send_cartesian_pose(debug_pose)
            self.fam.close_gripper()
            self.fam.test_send_joint_angles(self.joint_angles["look_at_ground_pose"])
            self.fam.test_send_joint_angles(self.joint_angles["intermediate_place_pose"])
            self.fam.test_send_joint_angles(self.joint_angles["place_pose"])
            self.fam.open_gripper()
            self.fam.test_send_joint_angles(self.joint_angles["intermediate_place_pose"])
            self.fam.test_send_joint_angles(self.joint_angles["look_at_ground_pose"])
        if msg.data == 'e_stop':
            self.perception_pose = None

    def setup_arm_for_pick(self):
        """Setup the arm to go to pick pose
        :returns: None

        """
        self.fam.example_clear_faults()
        # self.fam.test_send_joint_angles(self.joint_angles["vertical_pose"])
        self.fam.test_send_joint_angles(self.joint_angles["look_at_ground_pose"])
        self.fam.open_gripper()

if __name__ == "__main__":
    rospy.init_node('pick_and_place')
    PAP = PickAndPlace()
    rospy.spin()

