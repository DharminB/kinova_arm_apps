#!/usr/bin/env python
from __future__ import print_function

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from kinova_arm_apps.full_arm_movement import FullArmMovement

class PickAndPlace(object):

    """pick and place code using full arm movement and 3d segmentation perception"""

    def __init__(self):
        self.fam = FullArmMovement()

        # Subscribers
        self.perception_pose_sub = rospy.Subscriber('~pose_in', PoseStamped, self.perception_pose_cb)
        self.event_in_sub = rospy.Subscriber('~event_in', String, self.event_in_cb)

        self.setup_arm_for_pick()

    def perception_pose_cb(self, msg):
        self.perception_pose = msg

    def event_in_cb(self, msg):
        if msg.data == 'e_start':
            pass
        if msg.data == 'e_stop':
            pass

    def setup_arm_for_pick(self):
        """Setup the arm to go to pick pose
        :returns: None

        """
        self.fam.example_clear_faults()
        vertical_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        intermediate_pose = [179.0, 17.0, 182.0, 231.0, 0.0, 56.0, 92.0]
        look_at_ground_pose = [179.0, 34.0, 182.0, 280.0, 0.0, 295.0, 92.0]

        # self.fam.test_send_joint_angles(vertical_pose)
        # self.fam.test_send_joint_angles(intermediate_pose)
        self.fam.test_send_joint_angles(look_at_ground_pose)
        self.fam.open_gripper()
        self.fam.close_gripper()

if __name__ == "__main__":
    rospy.init_node('pick_and_place')
    PAP = PickAndPlace()
    rospy.spin()

