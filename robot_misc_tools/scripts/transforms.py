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
    tf_msg.transform.translation.x = pose_msg.position.x
    tf_msg.transform.translation.y = pose_msg.position.y
    tf_msg.transform.translation.z = pose_msg.position.z
    tf_msg.transform.rotation.x = pose_msg.orientation.x
    tf_msg.transform.rotation.y = pose_msg.orientation.y
    tf_msg.transform.rotation.z = pose_msg.orientation.z
    tf_msg.transform.rotation.w = pose_msg.orientation.w

    # Publish transform
    b.sendTransform(tf_msg)

def camera_angle_callback(pose_msg):

    # Create broadcaster
    b = tf2_ros.TransformBroadcaster()

    # Create transform message
    tf_msg = tf2_ros.TransformStamped()
    tf_msg.header.stamp = rospy.Time.now()
    tf_msg.header.frame_id = "aruco_camera"
    tf_msg.child_frame_id = "robot_frame"
    tf_msg.transform.translation.x = 0 #define these with CAD files
    tf_msg.transform.translation.y = 0 #define these with CAD files
    tf_msg.transform.translation.z = 0 #define these with CAD files

    tf_msg.transform.rotation.x = pose_msg.orientation.x
    tf_msg.transform.rotation.y = pose_msg.orientation.y
    tf_msg.transform.rotation.z = pose_msg.orientation.z
    tf_msg.transform.rotation.w = pose_msg.orientation.w

    # Publish transform
    b.sendTransform(tf_msg)

# getStaticTransform returns a TransformStamped filled with values defined in parameter server
# Parameter server key should be /transform/{TRANSFORM_NAME}
def getStaticTransform(transformList):

    staticTransformList = []

    for transform in transformList:
        tf = tf2_ros.TransformStamped()
        tf.header.stamp = rospy.Time.now()

        # Read in frame ids from parameter server
        tf.header.frame_id = transform["frame_id"]
        tf.child_frame_id = transform["child_id"]

        print tf.header.frame_id
        print tf.child_frame_id

        # Read in translation fro_m parameter server
        tf.transform.translation.x = transform["trans_x"]
        tf.transform.translation.y = transform["trans_y"]
        tf.transform.translation.z = transform["trans_z"]

        # Read in rotation from parameter server
        tf.transform.rotation.x = transform["rot_x"]
        tf.transform.rotation.y = transform["rot_y"]
        tf.transform.rotation.z = transform["rot_z"]
        tf.transform.rotation.w = transform["rot_w"]

        staticTransformList.append(tf)
    return staticTransformList


if __name__ == "__main__":

    # Initialize ROS node
    rospy.init_node("transform_publisher")

    # 'world' frame assumed to be the base of the collection bin (side by the outer wall,
    # closest to the corner of the competition area)


    # Marker board transform (relative to world - FIXED)

    # Read in transform file from parameter server
    transformFile = rospy.get_param("/transform_file", default="static_transforms.json")
    with open(transformFile) as f:
        transformDict = json.load(f)

    # Extract list of transform objects
    transformList = transformDict["transforms"]


    broadcaster = tf2_ros.StaticTransformBroadcaster()
    broadcaster.sendTransform(getStaticTransform(transformList))


    # Aruco marker camera transform (relative to marker board)
    rospy.Subscriber("ekf/pose_filtered", Pose, aruco_pose_callback)

    # Aruco marker camera base transform (relative to marker camera)
    rospy.Subscriber("camera_angle", Pose, camera_angle_callback)

    rospy.spin()