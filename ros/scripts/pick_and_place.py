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

    vertical_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    intermediate_pick_pose = [179.0, 17.0, 182.0, 231.0, 0.0, 56.0, 92.0]
    look_at_ground_pose = [179.0, 34.0, 182.0, 280.0, 0.0, 295.0, 92.0]
    intermediate_place_pose = [257.0, 11.0, 182.0, 241.0, 0.0, 310.0, 85.0]
    place_pose = [257.0, 31.0, 182.0, 229.0, 0.0, 342.0, 85.0]
    def __init__(self):
        self.fam = FullArmMovement()

        # Subscribers
        self.perception_pose_sub = rospy.Subscriber('~pose_in', PoseStamped, self.perception_pose_cb)
        self.event_in_sub = rospy.Subscriber('~event_in', String, self.event_in_cb)
        self.debug_pose_pub = rospy.Publisher('~debug_pose', PoseStamped, queue_size=1)

        self.setup_arm_for_pick()

    def perception_pose_cb(self, msg):
        x_min, x_max = -0.7, -0.4
        y_min, y_max = -0.2, 0.2
        z_min, z_max = 0.0, 0.1
        rospy.loginfo(msg.pose.position)
        if x_min < msg.pose.position.x < x_max and \
                y_min < msg.pose.position.y < y_max and \
                z_min < msg.pose.position.z < z_max:
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
            self.fam.test_send_joint_angles(self.look_at_ground_pose)
            self.fam.test_send_joint_angles(self.intermediate_place_pose)
            self.fam.test_send_joint_angles(self.place_pose)
            self.fam.open_gripper()
            self.fam.test_send_joint_angles(self.intermediate_place_pose)
            self.fam.test_send_joint_angles(self.look_at_ground_pose)
        if msg.data == 'e_stop':
            self.perception_pose = None

    def setup_arm_for_pick(self):
        """Setup the arm to go to pick pose
        :returns: None

        """
        self.fam.example_clear_faults()
        # self.fam.test_send_joint_angles(self.vertical_pose)
        # self.fam.test_send_joint_angles(self.intermediate_pick_pose)
        self.fam.test_send_joint_angles(self.look_at_ground_pose)
        self.fam.open_gripper()
        # self.fam.close_gripper()

if __name__ == "__main__":
    rospy.init_node('pick_and_place')
    PAP = PickAndPlace()
    rospy.spin()

