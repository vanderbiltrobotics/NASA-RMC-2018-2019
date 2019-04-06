#!/usr/bin/env python
import rospy
import numpy as np
import tf2_ros
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion
import cv2


    

class Field:
    def __init__(self, length, width):
                     self.map = np.array(size=(600,400))
                     self.xpos = 0
                     self.ypos = 0
                     self.tfBuffer = tf2_ros.Buffer()
                     self.listener = tf2_ros.TransformListener(tfBuffer)
                     #self.map_pub = rospy.Publisher

    def update_field(self, new_map_msg):
                     #convert msg to numpy array
                     #get pose of robot relative to world
                     #update corresponding region of self.map

        
                     local_map = np.reshape(new_map_msg.data, (new_map_msg.info.height, new_map_msg.info.width))
                     cur_tf = tfBuffer.lookup_transform("world_pose", "robot_pose", rospy.time(0))
                     q = [cur_tf.transform.rotation.x,
                          cur_tf.transform.rotation.y,
                          cur_tf.transform.rotation.z,
                          cur_tf.transform.rotation.w]  #quaternion from tf
                     rpy = euler_from_quaternion(Quaternion(q[0], q[1], q[2], q[3]))
                     #Rotate the local map based on the difference in angles of the transforms
                     (h, w) = local_map.shape[:2]
                     (origin_X, origin_Y) = (w/2, h/2)

                     
                     M = cv2.getRotationMatrix2D((cX, cY), rpy[2], 1.0)
                     cos = np.abs(M[0, 0])
                     sin = np.abs(M[0, 1])
                     
                     new_w = int((h * sin) + (w * cos))
                     new_h = int((h * cos) + (w * sin))
                     
                     M[0, 2] += (new_w/2) - origin_X
                     M[1, 2] += (new_h/2) - origin_Y
                     adj_lmap = cv2.warpAffine(local_map, M, (new_w, new_h))

                     #Add values from adjusted local map to world map

                     (adj_x, adj_y) = adj_lmap.shape[:2]
                     
                     
                     local_x = cur_tf.getOrigin().x()
                     local_y = cur_tf.getOrigin().y()
                     (field_x, field_y) = Field.shape[:2]
                     field_x /= 2
                     field_y /= 2
                     dif_x = np.abs(field_x - local_x)
                     dif_y = np.abs(field_y - local_y)

                     Field[dif_x:(dif_x+adj_x), dif_y:(dif_y+adj_y)] = adj_lmap
                                                 
                             
                    

if __name__ == "__main__":
    field = Field(length = 600, width = 400)
    #listener = tf.transform_listener()
    rospy.Subscriber("elevation_mapping/occ_grid", OccupancyGrid, field.update_field)
    rospy.spin()
                    
                     
