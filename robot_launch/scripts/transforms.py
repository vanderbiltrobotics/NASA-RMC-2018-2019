#!/usr/bin/env python

# Import ROS packages
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped


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
    rospy.Subscriber("cur_pose", PoseStamped, aruco_pose_callback)

    # Aruco marker camera base transform (relative to marker camera)

    # Robot center transform (relative to Aruco camera base)
    # Robot front_left (relative to robot center)
    # Robot front_right (relative to robot center)
    # Robot back_left (relative to robot center)
    # Robot back_right (relative to robot center)

    # Kinect base transform (relative to robot center)
    # (OTHER KINECT FRAMES AUTOMATICALLY PUBLISHED RELATIVE TO KINECT BASE)

    rospy.spin()