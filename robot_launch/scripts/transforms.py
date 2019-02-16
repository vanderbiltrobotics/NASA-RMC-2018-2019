#!/usr/bin/env python

# Import ROS packages
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose


def aruco_pose_callback(pose_msg):

    # Create broadcaster
    b = tf2_ros.TransformBroadcaster()

    # Create transform message
    tf_msg = tf2_ros.TransformStamped()
    tf_msg.header.stamp = rospy.Time.now()
    tf_msg.header.frame_id = "aruco_board_origin"
    tf_msg.child_frame_id = "aruco_camera"
    tf_msg.transform.translation.x = pose_msg.pose.position.x
    tf_msg.transform.translation.y = pose_msg.pose.position.y
    tf_msg.transform.translation.z = pose_msg.pose.position.z
    tf_msg.transform.rotation.x = pose_msg.pose.orientation.x
    tf_msg.transform.rotation.y = pose_msg.pose.orientation.y
    tf_msg.transform.rotation.z = pose_msg.pose.orientation.z
    tf_msg.transform.rotation.w = pose_msg.pose.orientation.w

    # Publish transform
    b.sendTransform(tf_msg)

def camera_angle_callback(pose_msg):

    # Create broadcaster
    b = tf2_ros.TransformBroadcaster()

    # Create transform message
    tf_msg = tf2_ros.TransformStamped()
    tf_msg.header.stamp = rospy.Time.now()
    tf_msg.header.frame_id = "robot_frame"
    tf_msg.child_frame_id = "aruco_camera"
    tf_msg.transform.translation.x = 0 #define these with CAD files
    tf_msg.transform.translation.y = 0 #define these with CAD files
    tf_msg.transform.translation.z = 0 #define these with CAD files

    tf_msg.transform.rotation.x = pose_msg.pose.orientation.x
    tf_msg.transform.rotation.y = pose_msg.pose.orientation.y
    tf_msg.transform.rotation.z = pose_msg.pose.orientation.z
    tf_msg.transform.rotation.w = pose_msg.pose.orientation.w

    # Publish transform
    b.sendTransform(tf_msg)

if __name__ == "__main__":

    # Initialize ROS node
    rospy.init_node("transform_publisher")

    # 'world' frame assumed to be the base of the collection bin (side by the outer wall,
    # closest to the corner of the competition area)

    # Marker board transform (relative to world - FIXED)
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    board_tf = tf2_ros.TransformStamped()
    board_tf.header.stamp = rospy.Time.now()
    board_tf.header.frame_id = "world"
    board_tf.child_frame_id = "aruco_board_origin"
    board_tf.transform.translation.x = 0.2
    board_tf.transform.translation.y = 0.1
    board_tf.transform.translation.z = 0.75
    board_tf.transform.rotation.w = 1.0
    broadcaster.sendTransform(board_tf)

    # Aruco marker camera transform (relative to marker board)
    rospy.Subscriber("ekf/pose_filtered", Pose, aruco_pose_callback)

    # Aruco marker camera base transform (relative to marker camera)
    rospy.Subscriber("camera_angle", Pose, camera_angle_callback)

    # Robot front left (relative to robot center)
    robot_front_left_tf = tf2_ros.TransformStamped()
    robot_front_left_tf.header.stamp = rospy.Time.now()
    robot_front_left_tf.header.frame_id = "robot_front_left"
    robot_front_left_tf.child_frame_id = "robot_frame"
    robot_front_left_tf.transform.translation.x = 0   #define these with CAD
    robot_front_left_tf.transform.translation.y = 0   #define these with CAD
    robot_front_left_tf.transform.translation.z = 0  #define these with CAD
    robot_front_left_tf.transform.rotation.w = 1.0
    broadcaster.sendTransform(robot_front_left_tf)

    # Robot front_right (relative to robot center)
    robot_front_right_tf = tf2_ros.TransformStamped()
    robot_front_right_tf.header.stamp = rospy.Time.now()
    robot_front_right_tf.header.frame_id = "robot_front_right"
    robot_front_right_tf.child_frame_id = "robot_frame"
    robot_front_right_tf.transform.translation.x = 0  # define these with CAD
    robot_front_right_tf.transform.translation.y = 0  # define these with CAD
    robot_front_right_tf.transform.translation.z = 0  # define these with CAD
    robot_front_right_tf.transform.rotation.w = 1.0
    broadcaster.sendTransform(robot_front_right_tf)

    # Robot back_left (relative to robot center)
    robot_back_left_tf = tf2_ros.TransformStamped()
    robot_back_left_tf.header.stamp = rospy.Time.now()
    robot_back_left_tf.header.frame_id = "robot_back_left"
    robot_back_left_tf.child_frame_id = "robot_frame"
    robot_back_left_tf.transform.translation.x = 0  # define these with CAD
    robot_back_left_tf.transform.translation.y = 0  # define these with CAD
    robot_back_left_tf.transform.translation.z = 0  # define these with CAD
    robot_back_left_tf.transform.rotation.w = 1.0
    broadcaster.sendTransform(robot_back_left_tf)

    # Robot back_right (relative to robot center)
    robot_back_right_tf = tf2_ros.TransformStamped()
    robot_back_right_tf.header.stamp = rospy.Time.now()
    robot_back_right_tf.header.frame_id = "robot_back_right"
    robot_back_right_tf.child_frame_id = "robot_frame"
    robot_back_right_tf.transform.translation.x = 0  # define these with CAD
    robot_back_right_tf.transform.translation.y = 0  # define these with CAD
    robot_back_right_tf.transform.translation.z = 0  # define these with CAD
    robot_back_right_tf.transform.rotation.w = 1.0
    broadcaster.sendTransform(robot_back_right_tf)

    # Kinect base transform (relative to robot center)
    robot_kinect_tf = tf2_ros.TransformStamped()
    robot_kinect_tf.header.stamp = rospy.Time.now()
    robot_kinect_tf.header.frame_id = "robot_kinect"
    robot_kinect_tf.child_frame_id = "robot_frame"
    robot_kinect_tf.transform.translation.x = 0  # define these with CAD
    robot_kinect_tf.transform.translation.y = 0  # define these with CAD
    robot_kinect_tf.transform.translation.z = 0  # define these with CAD
    robot_kinect_tf.transform.rotation.w = 1.0
    broadcaster.sendTransform(robot_kinect_tf)

    # (OTHER KINECT FRAMES AUTOMATICALLY PUBLISHED RELATIVE TO KINECT BASE)

    rospy.spin()