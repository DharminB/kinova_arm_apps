#!/usr/bin/env python
from __future__ import print_function

import tf
import math
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from kortex_driver.srv import *
from kortex_driver.msg import *

class FullArmMovement:
    def __init__(self):

        self.HOME_ACTION_IDENTIFIER = 2

        # Get node params
        self.robot_name = rospy.get_param('~robot_name', "my_gen3")
        self.degrees_of_freedom = rospy.get_param("/" + self.robot_name + "/degrees_of_freedom", 7)
        self.is_gripper_present = rospy.get_param("/" + self.robot_name + "/is_gripper_present", False)

        rospy.loginfo("Using robot_name " + self.robot_name + " , robot has " + str(self.degrees_of_freedom) + " degrees of freedom and is_gripper_present is " + str(self.is_gripper_present))

        # Init the services
        clear_faults_full_name = '/' + self.robot_name + '/base/clear_faults'
        rospy.wait_for_service(clear_faults_full_name)
        self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

        read_action_full_name = '/' + self.robot_name + '/base/read_action'
        rospy.wait_for_service(read_action_full_name)
        self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)

        execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
        rospy.wait_for_service(execute_action_full_name)
        self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

        set_cartesian_reference_frame_full_name = '/' + self.robot_name + '/control_config/set_cartesian_reference_frame'
        rospy.wait_for_service(set_cartesian_reference_frame_full_name)
        self.set_cartesian_reference_frame = rospy.ServiceProxy(set_cartesian_reference_frame_full_name, SetCartesianReferenceFrame)

        play_cartesian_trajectory_full_name = '/' + self.robot_name + '/base/play_cartesian_trajectory'
        rospy.wait_for_service(play_cartesian_trajectory_full_name)
        self.play_cartesian_trajectory = rospy.ServiceProxy(play_cartesian_trajectory_full_name, PlayCartesianTrajectory)

        play_joint_trajectory_full_name = '/' + self.robot_name + '/base/play_joint_trajectory'
        rospy.wait_for_service(play_joint_trajectory_full_name)
        self.play_joint_trajectory = rospy.ServiceProxy(play_joint_trajectory_full_name, PlayJointTrajectory)

        send_gripper_command_full_name = '/' + self.robot_name + '/base/send_gripper_command'
        rospy.wait_for_service(send_gripper_command_full_name)
        self.send_gripper_command = rospy.ServiceProxy(send_gripper_command_full_name, SendGripperCommand)

        activate_pub_of_action_topic_full_name = '/' + self.robot_name + '/base/activate_publishing_of_action_topic'
        rospy.wait_for_service(activate_pub_of_action_topic_full_name)
        self.activate_pub_of_action_topic = rospy.ServiceProxy(activate_pub_of_action_topic_full_name, OnNotificationActionTopic)

        stop_action_full_name = '/' + self.robot_name + '/base/stop_action'
        rospy.wait_for_service(stop_action_full_name)
        self.stop_action = rospy.ServiceProxy(stop_action_full_name, StopAction)

        self.base_feedback = None
        self.base_feedback_sub = rospy.Subscriber('/' + self.robot_name + '/base_feedback', BaseCyclic_Feedback, self.base_feedback_cb)

        self.action_notification = None
        self.action_sub = rospy.Subscriber('/' + self.robot_name + '/action_topic', ActionNotification, self.action_notification_cb)

    def base_feedback_cb(self, msg):
        self.current_ee_pose = (msg.base.commanded_tool_pose_x,
                                msg.base.commanded_tool_pose_y,
                                msg.base.commanded_tool_pose_z,
                                msg.base.commanded_tool_pose_theta_x,
                                msg.base.commanded_tool_pose_theta_y,
                                msg.base.commanded_tool_pose_theta_z)

    def action_notification_cb(self, msg):
        self.action_notification = msg

    def start_action_topic(self):
        try:
            self.activate_pub_of_action_topic()
            rospy.sleep(2)
            return True
        except rospy.ServiceException:
            rospy.logerr("Failed to call OnNotificationActionTopic")
            return False

    def stop_current_action(self):
        try:
            self.stop_action()
        except rospy.ServiceException:
            rospy.logerr("Failed to call StopAction")
        else:
            rospy.loginfo("Stopped current action")

    def example_clear_faults(self):
        try:
            self.clear_faults()
        except rospy.ServiceException:
            rospy.logerr("Failed to call ClearFaults")
        else:
            rospy.loginfo("Cleared the faults successfully")
            rospy.sleep(2.5)

    def example_set_cartesian_reference_frame(self):
        # Prepare the request with the frame we want to set
        req = SetCartesianReferenceFrameRequest()
        req.input.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_MIXED

        # Call the service
        try:
            self.set_cartesian_reference_frame()
        except rospy.ServiceException:
            rospy.logerr("Failed to call SetCartesianReferenceFrame")
        else:
            rospy.loginfo("Set the cartesian reference frame successfully")

        # Wait a bit
        rospy.sleep(0.25)

    def send_cartesian_pose(self, pose=None):
        self.example_set_cartesian_reference_frame()
        if pose is None:
            pose = self.get_pose_from_current_ee_pose()

        # feedback = rospy.wait_for_message("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback)
        # print(feedback.base)
        print(self.current_ee_pose)

        req = PlayCartesianTrajectoryRequest()
        # req.input.target_pose.x = feedback.base.commanded_tool_pose_x
        # req.input.target_pose.y = feedback.base.commanded_tool_pose_y
        # req.input.target_pose.z = feedback.base.commanded_tool_pose_z + 0.15
        # req.input.target_pose.theta_x = feedback.base.commanded_tool_pose_theta_x
        # req.input.target_pose.theta_y = feedback.base.commanded_tool_pose_theta_y
        # req.input.target_pose.theta_z = feedback.base.commanded_tool_pose_theta_z

        req.input.target_pose.x = pose.pose.position.x
        req.input.target_pose.y = pose.pose.position.y
        req.input.target_pose.z = pose.pose.position.z
        theta_x, theta_y, theta_z = tf.transformations.euler_from_quaternion((
                pose.pose.orientation.x, pose.pose.orientation.y,
                pose.pose.orientation.z, pose.pose.orientation.w))
        print(theta_x, theta_y, theta_z)
        req.input.target_pose.theta_x = math.degrees(theta_x)
        req.input.target_pose.theta_y = math.degrees(theta_y)
        req.input.target_pose.theta_z = math.degrees(theta_z)

        pose_speed = CartesianSpeed()
        pose_speed.translation = 0.05
        pose_speed.orientation = 15

        # The constraint is a one_of in Protobuf. The one_of concept does not exist in ROS
        # To specify a one_of, create it and put it in the appropriate list of the oneof_type member of the ROS object : 
        req.input.constraint.oneof_type.speed.append(pose_speed)

        # Call the service
        rospy.loginfo("Sending the robot to the cartesian pose...")
        try:
            self.play_cartesian_trajectory(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call PlayCartesianTrajectory")
        still_in_motion = True
        while still_in_motion:
            if self.action_notification is not None:
                still_in_motion = self.action_notification.action_event == ActionEvent.ACTION_START
            rospy.sleep(1.0)
        self.action_notification = None
        rospy.loginfo("Motion finished")

    def test_send_joint_angles(self, joint_angles):
        # Create the list of angles
        if not isinstance(joint_angles, list):
            rospy.logerr("Joint angles not provided in a list")
            return
        if len(joint_angles) != self.degrees_of_freedom:
            rospy.logerr("Joint angles has mismatched joints")
            return
        all_float = all([isinstance(i, float) for i in joint_angles])
        if not all_float:
            rospy.logerr("Joint angles has a non float value")
            return

        req = PlayJointTrajectoryRequest()
        for i in range(self.degrees_of_freedom):
            temp_angle = JointAngle() 
            temp_angle.joint_identifier = i
            temp_angle.value = joint_angles[i]
            req.input.joint_angles.joint_angles.append(temp_angle)
        
        # Send the angles
        rospy.loginfo("Sending the robot to given angle...")
        try:
            self.play_joint_trajectory(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call PlayJointTrajectory")
        still_in_motion = True
        start_time = rospy.get_time()
        while still_in_motion:
            try:
                print(self.action_notification.action_event)
            except Exception as e:
                pass
            if self.action_notification is not None:
                still_in_motion = self.action_notification.action_event == ActionEvent.ACTION_START
            rospy.sleep(1.0)
            if rospy.get_time() - 10.0 > start_time:
                break
        self.action_notification = None
        rospy.loginfo("Motion finished")

    def example_send_gripper_command(self, value):
        req = SendGripperCommandRequest()
        finger = Finger()
        finger.finger_identifier = 0
        finger.value = value
        req.input.gripper.finger.append(finger)
        req.input.mode = GripperMode.GRIPPER_POSITION

        rospy.loginfo("Sending the gripper command...")

        # Call the service 
        try:
            self.send_gripper_command(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SendGripperCommand")

    def close_gripper(self):
        self.example_send_gripper_command(0.8)
        rospy.sleep(2.0)

    def open_gripper(self):
        self.example_send_gripper_command(0.0)
        rospy.sleep(2.0)

    @staticmethod
    def quat_from_rpy(roll, pitch, yaw):
        quat = Quaternion()
        quat.x, quat.y, quat.z, quat.w = tf.transformations.quaternion_from_euler(roll, pitch, yaw) 
        return quat

    def get_pose_from_current_ee_pose(self):
        pose = PoseStamped()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = self.current_ee_pose[0:3]
        pose.pose.orientation = FullArmMovement.quat_from_rpy(*map(math.radians, self.current_ee_pose[3:]))
        return pose
