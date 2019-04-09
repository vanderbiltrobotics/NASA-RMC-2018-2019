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
                     
                     #Add 1 to each valid point to differentiate from the later junk values
                     local_map += 1
                     
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
                     
                     #dif = difference in the origins between local map and world map
                     dif_x = cur_tf.transform.translation.x
                     dif_y = cur_tf.transform.translation.y
                     corner_x = dif_x - (adj_x/2)
                     corner_y = dif_y - (adj_x/2)
                     
                     #Take the greater value between the given field
                     self.map[corner_x:(corner_x+adj_x), corner_y:(corner_y+adj_y)] = np.maximum(self.map[corner_x:(corner_x+adj_x), corner_y:(corner_y+adj_y)], adj_lmap)
                     #Alternative: specifically for excluding 0 - probably better for our purposes. Didn't work well with images when testing.
                     #np.place(self.map[corner_x:(corner_x+adj_x), corner_y:(corner_y+adj_y)], adj_lmap > 0, adj_lmap)
                             
                    

if __name__ == "__main__":
    field = Field(length = 600, width = 400)
    #listener = tf.transform_listener()
    rospy.Subscriber("elevation_mapping/occ_grid", OccupancyGrid, field.update_field)
    rospy.spin()
                    
                     
