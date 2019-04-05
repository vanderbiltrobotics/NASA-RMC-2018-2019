#!/usr/bin/env python
import rospy
import numpy as np
import tf2_ros

field = Field(length = 600, width = 400)
#listener = tf.transform_listener()
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
rospy.Subscriber("elevation_mapping/occ_grid", OccupancyGrid, field.update_field)
rospy.spin()

    

class Field:
    def __init__(self, length, width):
                     self.map = np.array(size=(600,400)) #length, width
                     self.xpos = 0
                     self.ypos = 0
                     #self.map_pub = rospy.Publisher

    def update_field(self, new_map_msg):
                     #convert msg to numpy array
                     #get pose of robot relative to world
                     #update corresponding region of self.map
                     local_map = np.reshape(OccupancyGrid.data, (OccupancyGrid.info.height, OccupancyGrid.info.width))
                     cur_tf = tfBuffer.lookup_transform("world_pose", "robot_pose", rospy.time(0))
                     local_x = cur_tf.getOrigin().x()
                     local_y = cur_tf.getOrigin().y()
                     for y in local_map:
                         local_y = cur_tf.getOrigin().y()
                         for x in y:
                             self.map[local_y][local_x] = local_map[y][x]
                             local_y += 1
                         local_x += 1
                             
                     

        
    def msg_callback(new_map.msg):
                    
                     
