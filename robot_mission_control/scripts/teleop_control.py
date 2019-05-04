#!/usr/bin/env python

# ----------------------------------------------- #
# Table of index number of /joy.buttons:
# Index   Button name on the actual controller
#
# 0   A (Lead Screw)
# 1   B (Buckets)
# 2   X (Hinge)
# 3   Y (Conveyor)
# 4   LB
# 5   RB
# 6   back (accel_decel)
# 7   start (direction)
# 8   power
# 9   Button stick left
# 10  Button stick right

# Table of index number of /joy.axes:
# Index   Axis name on the actual controller
#
# 0   Moving left joystick left (+) and right (-) changes rotational velocity
# 1   Moving left joystick up (+) and down (-) changes linear velocity
# 2   LT
# 3   Left/Right Axis stick right
# 4   Up/Down Axis stick right
# 5   RT
# 6   cross key left/right      (buckets / hinge accel)
# 7   cross key up/down         (hinge / lead screw accel)
# ----------------------------------------------- #

# import required packages
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64, String


class TeleopControl:
    
    # Constructor
    def __init__(self):
        
        # Initialize drive speed publishers
        self.publishers = {
            "pub_drive_cmd": rospy.Publisher('drive_cmd', Twist, queue_size=0),
            "pub_hng": rospy.Publisher("dig_motor_speeds/hinge", Float64, queue_size=0),
            "pub_lsc": rospy.Publisher("dig_motor_speeds/lead_screw", Float64, queue_size=0),
            "pub_bkt": rospy.Publisher("dig_motor_speeds/bucket_chain", Float64, queue_size=0),
            "pub_cnv": rospy.Publisher("dig_motor_speeds/conveyor", Float64, queue_size=0),
        }

        self.state = {
            "acc_dir": 1.0,
            "acc_dir_toggle": 0,
            "acc_step": 0.05,
            "hng_on": False,
            "lsc_on": False,
            "bkt_on": False,
            "cnv_on": False,
            "hng_out": 0.0,
            "lsc_out": 0.0,
            "bkt_out": 0.0,
            "cnv_out": 0.0,
            "hng_toggle": 0,
            "lsc_toggle": 0,
            "bkt_toggle": 0,
            "cnv_toggle": 0,
            "hng_acc_toggle": 0,
            "lsc_acc_toggle": 0,
            "bkt_acc_toggle": 0,
            "cnv_acc_toggle": 0,
            "hng_start": 0.1,
            "lsc_start": 0.1,
            "bkt_start": 0.1,
            "cnv_start": 0.1
        }

        # Initialize subscribers
        self.joy_sub = rospy.Subscriber("joy", Joy, self.process_joystick_data)

    # Update the speed of a dig motor from joystick data
    def update_dig_motor(self, id, toggle, acc_toggle):

        # Check if we should change on / off
        if self.state[id + "_toggle"] == 1 and toggle == 0:

            # Toggle on / off
            self.state[id + "_on"] = not self.state[id + "_on"]

            # If turning on, set speed to default, else set to 0
            self.state[id + "_out"] = self.state[id + "_start"] if self.state[id + "_on"] else 0.0

        # Check if we should accel / decel
        if self.state[id + "_on"]:
            if self.state[id + "_acc_toggle"] != 0 and not self.check_button(acc_toggle, id):

                # Apply increment
                new_vel = self.state[id + "_out"] + self.state["acc_dir"] * self.state["acc_step"]

                # Limit speed value
                self.state[id + "_out"] = min(1.0, max(-1.0, new_vel))

        # Update toggle values
        self.state[id + "_toggle"] = toggle
        self.state[id + "_acc_toggle"] = acc_toggle if self.check_button(acc_toggle, id) else 0.0

    # Checks correspondence between accel buttons and dig motor ids
    def check_button(self, accel, id):

        # Check if accel value is in corresponding group
        if id in ["cnv", "hng"] and accel == 1:
            return True
        if id in ["bkt", "lsc"] and accel == -1:
            return True

        # Doesn't match
        return False

    # --- CALLBACK FUNCTIONS --- #

    # Callback for joystick data
    def process_joystick_data(self, msg):

        # Get motor velocity commands
        lin_vel = msg.axes[1]
        ang_vel = msg.axes[3]

        # Get direction and accel_decel toggle states
        acc_dir_toggle = msg.buttons[5]

        # Get dig motor toggle button states
        hng_toggle = msg.buttons[2]
        lsc_toggle = msg.buttons[0]
        bkt_toggle = msg.buttons[1]
        cnv_toggle = msg.buttons[3]

        # Get dig motor accel button states
        bkt_hng_acc_toggle = msg.axes[6]
        lsc_cnv_acc_toggle = msg.axes[7]

        # Update accel_direction
        if self.state["acc_dir_toggle"] == 1 and acc_dir_toggle == 0:
            self.state["acc_dir"] *= -1.0

        # Update dir and acc toggle
        self.state["acc_dir_toggle"] = acc_dir_toggle

        # Update each dig motor speeds, toggles, accel_toggles
        self.update_dig_motor("hng", hng_toggle, bkt_hng_acc_toggle)
        self.update_dig_motor("bkt", bkt_toggle, bkt_hng_acc_toggle)
        self.update_dig_motor("lsc", lsc_toggle, lsc_cnv_acc_toggle)
        self.update_dig_motor("cnv", cnv_toggle, lsc_cnv_acc_toggle)

        # --- PUBLISH COMMAND MESSAGES --- #

        # Drive twist
        drive_msg = Twist()
        drive_msg.linear.x = lin_vel
        drive_msg.angular.z = ang_vel
        self.publishers["pub_drive_cmd"].publish(drive_msg)

        # Dig motor speeds
        for id in ["hng", "lsc", "bkt", "cnv"]:

            # Create message
            dig_msg = Float64()
            dig_msg.data = self.state[id + "_out"]

            # Publish
            self.publishers["pub_" + id].publish(dig_msg)


if __name__ == '__main__':
    
    # Initialize as ROS node
    rospy.init_node('teleop_control')

    # Create a TeleopControl object 
    control = TeleopControl()

    # Ready to go
    rospy.loginfo("Teleop Control initialized...")

    # Loop continuously
    rospy.spin()