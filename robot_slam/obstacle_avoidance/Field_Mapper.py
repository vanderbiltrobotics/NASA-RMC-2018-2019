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

    #convert msg to numpy array
    #get pose of robot relative to world
    #update corresponding region of self.map
    def update_field(self, new_map_msg):

        
                     local_map = np.reshape(new_map_msg.data, (new_map_msg.info.height, new_map_msg.info.width))
                     cur_tf = tfBuffer.lookup_transform("world_pose", "robot_pose", rospy.time(0))
                     q = [cur_tf.transform.rotation.x,
                          cur_tf.transform.rotation.y,
                          cur_tf.transform.rotation.z,
                          cur_tf.transform.rotation.w]  #quaternion from tf
                     rpy = euler_from_quaternion(Quaternion(q[0], q[1], q[2], q[3])) #need yaw for the angle of the camera
                     
                     #Add 1 to each valid 0 point to differentiate from the later junk values
                     local_map[local_map == 0] += 1
                     
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
                     #corners of the local map relative to the full map
                     corner_xU = dif_x - (adj_x/2)
                     corner_yL = dif_y - (adj_x/2)
                     corner_xD = corner_xU + adj_x
                     corner_yR = corner_yL + adj_y
                     #corners of the local map relative to itself
                     lcorner_xU = 0
                     lcorner_yL = 0
                     lcorner_xD = adj_x
                     lcorner_yR = adj_y

                     #deals with values in local map that are in a row less than row 0
                     if corner_xU < 0:
                         lcorner_xU += (-corner_xU)
                         corner_xU = 0

                     #deals with values in local map that are in a column less than column 0
                     if corner_yL < 0:
                         lcorner_yL += (-corner_yL)
                         corner_yL = 0

                     (fX, fY) = self.map.shape[:2] # max values of full map

                     #deals with values in local map that are in a row greater than max row
                     if corner_xD > fX:
                         lcorner_xD -= (corner_xD - fX)
                         corner_xD = fX

                     #deals with values in local map that are in a column greater than max column
                     if corner_yR > fY:
                         lcorner_yR -= (corner_yR - fY)
                         corner_yR = fY

                         
            
                     #Transfer all valid values out of adjust local map and update their corresponding positions in the Field
                     np.copyto(self.map[corner_xU:corner_xD, corner_yL:corner_yR], adj_lmap[lcorner_xU:lcorner_xD, lcorner_yL:lcorner_yR], where=(adj_lmap != 0))
                             
                    

if __name__ == "__main__":
    field = Field(length = 600, width = 400)
    #listener = tf.transform_listener()
    rospy.Subscriber("elevation_mapping/occ_grid", OccupancyGrid, field.update_field)
    rospy.spin()
                    
                     
