#!/usr/bin/env python

# Import ROS packages
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
import json



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

# getStaticTransform returns a TransformStamped filled with values defined in parameter server
# Parameter server key should be /transform/{TRANSFORM_NAME}
def getStaticTransform(paramServerDict):
    tf = tf2_ros.TransformStamped()
    tf.stamp = rospy.Time.now()

    # Read in frame ids from parameter server
    tf.header.frame_id = paramServerDict["frame_id"]
    tf.child_frame_id = paramServerDict["child_frame_id"]

    # Read in translation from parameter server
    tf.transform.translation.x = paramServerDict["trans_x"]
    tf.transform.translation.y = paramServerDict["trans_y"]
    tf.transform.translation.z = paramServerDict["trans_z"]

    # Read in rotation from parameter server
    tf.transform.rotation.x = paramServerDict["rot_x"]
    tf.transform.rotation.y = paramServerDict["rot_y"]
    tf.transform.rotation.z = paramServerDict["rot_z"]
    tf.transform.rotation.w = paramServerDict["rot_w"]

    return tf


if __name__ == "__main__":

    # Initialize ROS node
    rospy.init_node("transform_publisher")

    # 'world' frame assumed to be the base of the collection bin (side by the outer wall,
    # closest to the corner of the competition area)


    # Marker board transform (relative to world - FIXED)
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    # Read in transform file from parameter server
    transformFile = rospy.get_param("/transform_file")
    with open("transformFile") as f:
        transformDict = json.load(f)

    # Extract list of transform objects
    transformList = transformDict["transforms"]

    for transform in transformList:
        broadcaster.sendTransform(getStaticTransform(transform))


    # Aruco marker camera transform (relative to marker board)
    rospy.Subscriber("ekf/pose_filtered", Pose, aruco_pose_callback)

    # Aruco marker camera base transform (relative to marker camera)
    rospy.Subscriber("camera_angle", Pose, camera_angle_callback)

    rospy.spin()