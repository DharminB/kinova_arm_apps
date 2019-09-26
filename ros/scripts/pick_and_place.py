#!/usr/bin/env python
from __future__ import print_function

import tf
import rospy
import copy
import math
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from kinova_arm_apps.full_arm_movement import FullArmMovement
import mcr_manipulation_measurers_ros.pose_transformer

class PickAndPlace(object):

    """pick and place code using full arm movement and 3d segmentation perception"""

    def __init__(self):
        self.fam = FullArmMovement()
        default_boundary_safety = {"x_min": -0.1, "x_max": 0.1, "y_min": -0.45,\
        		           "y_max": -0.25, "z_min": 0.0, "z_max": 0.1}
	self.boundary_safety = rospy.get_param("~boundary_safety ", default_boundary_safety )
	self.joint_angles = rospy.get_param("~joint_angles", {})
        self.perception_pose = None
        self.listener = tf.TransformListener()

        # how long to wait for transform (in seconds)
        self.wait_for_transform = 0.1

        self.transform_tries = 5

        # Subscribers
        self.perception_pose_sub = rospy.Subscriber('~pose_in', PoseStamped, self.perception_pose_cb)
        self.event_in_sub = rospy.Subscriber('~event_in', String, self.event_in_cb)
        self.debug_pose_pub = rospy.Publisher('~debug_pose', PoseStamped, queue_size=1)

        self.setup_arm_for_pick()
	rospy.loginfo("READY!")

    def perception_pose_cb(self, msg):
        msg = self.get_transformed_pose(msg, 'base_link')
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
            debug_pose.pose.position.z = self.perception_pose.pose.position.z + 0.01
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

    def get_transformed_pose(self, reference_pose, target_frame):
        """ Transform pose with multiple retries

        :return: The updated state.
        :rtype: str

        """
        for i in range(0, self.transform_tries):
            transformed_pose = self.transform_pose(reference_pose, target_frame)
            if transformed_pose:
                return transformed_pose
        transformed_pose = None
        return transformed_pose


    def transform_pose(self, reference_pose, target_frame):
        """
        Transforms a given pose into the target frame.

        :param reference_pose: The reference pose.
        :type reference_pose: geometry_msgs.msg.PoseStamped

        :param target_frame: The name of the taget frame.
        :type target_frame: String

        :return: The pose in the target frame.
        :rtype: geometry_msgs.msg.PoseStamped or None

        """
        try:
            common_time = self.listener.getLatestCommonTime(
                target_frame, reference_pose.header.frame_id
            )

            self.listener.waitForTransform(
                target_frame, reference_pose.header.frame_id,
                common_time, rospy.Duration(self.wait_for_transform)
            )
            reference_pose.header.stamp = common_time

            transformed_pose = self.listener.transformPose(
                target_frame, reference_pose,
            )

            return transformed_pose

        except tf.Exception, error:
            rospy.logwarn("Exception occurred: {0}".format(error))
            return None

if __name__ == "__main__":
    rospy.init_node('pick_and_place')
    PAP = PickAndPlace()
    rospy.spin()

