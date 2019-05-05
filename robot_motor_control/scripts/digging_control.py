#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, String, Int32, Bool


class DigManager:

    def __init__(self):

        # Current dig mode - "pct_out" or "vel_pos"
        self.control_mode = "pct_out"

        # Publishers for each dig motor
        self.publishers = {
            "pub_pct_hng": rospy.Publisher("hinge_motor/set_percent_output", Float64, queue_size=0),
            "pub_pct_lsc": rospy.Publisher("lead_screw_motor/set_percent_output", Float64, queue_size=0),
            "pub_pct_bkt": rospy.Publisher("bucket_motor/set_percent_output", Float64, queue_size=0),
            "pub_pct_cnv": rospy.Publisher("conveyor_motor/set_percent_output", Float64, queue_size=0),
            "pub_vel_lsc": rospy.Publisher("lead_screw_motor/set_velocity", Float64, queue_size=0),
            "pub_vel_bkt": rospy.Publisher("bucket_motor/set_velocity", Float64, queue_size=0),
            "pub_vel_cnv": rospy.Publisher("conveyor_motor/set_velocity", Float64, queue_size=0),
            "pub_pos_hng": rospy.Publisher("hinge_motor/set_position", Float64, queue_size=0),
        }

        # Lead screw limit switch and encoder information
        self.lsc_enc_pos = None
        self.lsc_bkd_lim = False

        # Create subscribers for relevant encoder data
        rospy.Subscriber("lead_screw/position", Int32, self.update_lsc_enc_pos)
        rospy.Subscriber("lead_screw/reverse_limit", Bool, self.update_lsc_bkd_lim)

        # Max velocity parameters for various motors
        self.max_vel_lsc = 16000.0
        self.max_vel_bkt = 700.0
        self.max_pos_hng = -881.0    # Fully retracted
        self.min_pos_hng = -725.0    # Digging point
        self.max_pos_lsc = 10000000
        self.min_pos_lsc = 0.0
        self.max_vel_cnv = 500.0
        self.cnv_vel_dig = 100.0
        self.cnv_vel_collect = 10.0
        self.cnv_vel_deposit = 450.0

        # Lead screw position parameters
        self.gravel_depth = 0.6
        self.max_depth = 0.95
        self.lsc_home_pos = 0.1

        # Messages storing current percent outputs for each motor
        self.hng_pct_msg = Float64()
        self.lsc_pct_msg = Float64()
        self.bkt_pct_msg = Float64()
        self.cnv_pct_msg = Float64()

        # Messages storing current velocity / speeds for each motor
        self.hng_pos_msg = Float64()
        self.lsc_vel_msg = Float64()
        self.bkt_vel_msg = Float64()
        self.cnv_vel_msg = Float64()

    # Callback for lead screw encoder position
    def update_lsc_enc_pos(self, msg):
        self.lsc_enc_pos = msg.data

    # Callback for lead screw limit switch
    def update_lsc_bkd_lim(self, msg):
        self.lsc_bkd_lim = msg.data

    # Callback for dig mode changes
    def update_control_mode(self, mode):
        self.control_mode = mode

    # Update current speed setting for each motor
    # Mask can be applied to only set speeds for specific motor(s)
    def update_pct_output(self, hng=0.0, lsc=0.0, bkt=0.0, cnv=0.0, update_mask=(1,1,1,1)):

        # Set message speeds
        if update_mask[0] == 1:
            self.hng_pct_msg.data = hng
        if update_mask[1] == 1:
            self.lsc_pct_msg.data = lsc
        if update_mask[2] == 1:
            self.bkt_pct_msg.data = bkt
        if update_mask[3] == 1:
            self.cnv_pct_msg.data = cnv

    # Publish messages to set speed of each digging motor
    def publish_pct_output(self):

        # Publish messages
        self.publishers["pub_pct_hng"].publish(self.hng_pct_msg)
        self.publishers["pub_pct_lsc"].publish(self.lsc_pct_msg)
        self.publishers["pub_pct_bkt"].publish(self.bkt_pct_msg)
        self.publishers["pub_pct_cnv"].publish(self.cnv_pct_msg)

    # --- UPDATE INDIVIDUAL MOTOR PCT OUTPUTS --- #

    # these callbacks will only do something if control mode is "pct_out"

    def update_pct_out_hng(self, new_pct_output):
        if self.control_mode == "pct_out":
            self.update_pct_output(hng=new_pct_output.data, update_mask=(1, 0, 0, 0))

    def update_pct_out_lsc(self, new_pct_output):
        if self.control_mode == "pct_out":
            self.update_pct_output(lsc=new_pct_output.data, update_mask=(0, 1, 0, 0))

    def update_pct_out_bkt(self, new_pct_output):
        if self.control_mode == "pct_out":
            self.update_pct_output(bkt=new_pct_output.data, update_mask=(0, 0, 1, 0))

    def update_pct_out_cnv(self, new_pct_output):
        if self.control_mode == "pct_out":
            self.update_pct_output(cnv=new_pct_output.data, update_mask=(0, 0, 0, 1))

    # --- UPDATE INDIVIDUAL MOTOR VELOCITIES OR POSITIONS --- #

    # Publish messages to set speed of each digging motor
    def publish_vel_pos(self):

        # Publish messages
        self.publishers["pub_pos_hng"].publish(self.hng_pos_msg)
        self.publishers["pub_vel_lsc"].publish(self.lsc_vel_msg)
        self.publishers["pub_vel_bkt"].publish(self.bkt_vel_msg)
        self.publishers["pub_vel_cnv"].publish(self.cnv_vel_msg)

    # Contiues publishing the specified motor velocities for a given duration
    def continue_for_duration(self, duration, loop_rate):

        # Get loop rate, store start time
        loop_rate = rospy.Rate(loop_rate)
        start_time = rospy.Time.now()

        # Continue for specified duration
        while rospy.Time.now() - start_time < duration:

            # Publish current speed settings and loop
            self.publish_vel_pos()
            loop_rate.sleep()

    # Moves hinge to a specific angle
    def set_hinge_position(self, pos):

        # Limit value from 0.0 to 1.0
        pos = min(1.0, max(0.0, pos))

        # Scale value to correct range
        pos = pos * (self.max_pos_hng - self.min_pos_hng) + self.min_pos_hng

        # Create message
        msg = Float64()
        msg.data = pos

        # Update message
        self.hng_pos_msg = msg

    # Set the velocity of the lead screw
    def set_lead_screw_velocity(self, vel):

        # Limit value from -1.0 to 1.0
        vel = min(1.0, max(-1.0, vel))

        # Scale value to correct range
        vel = vel * self.max_vel_lsc

        # Create message
        msg = Float64()
        msg.data = vel

        # Publish message
        self.lsc_vel_msg = msg

    # Scale lead screw encoder position to range 0 to 1
    def get_lsc_pos_scaled(self):
        return (self.lsc_enc_pos - self.min_pos_lsc) / self.max_pos_lsc

    # Move the lead screw to a specific position at a specific velocity
    def set_lead_screw_position(self, pos, vel):

        # Scale encoder value to range 0.0 to 1.0
        encoder_pos = self.get_lsc_pos_scaled()

        # Determine the direction we need to move
        direction = 1.0 if encoder_pos - pos > 0.0 else -1.0

        # Limit velocity from -1.0 to 1.0
        vel = min(1.0, max(-1.0, vel))
        vel = abs(vel) * direction

        # Set loop rate
        loop_rate = rospy.Rate(30)

        # Velocity command
        msg = Float64()
        msg.data = vel
        self.lsc_vel_msg = msg

        while self.get_lsc_pos_scaled() - pos > 0.01:

            # Publish the message and loop
            self.publish_vel_pos()
            loop_rate.sleep()

        # Set velocity to zero
        self.lsc_vel_msg = Float64()

    # Set the velocity of the bucket chain
    def set_bucket_chain_velocity(self, vel):

        # Limit value from 0.0 to 1.0
        vel = min(1.0, max(-1.0, vel))

        # Scale value to correct range
        vel = vel * self.max_vel_bkt

        # Create message
        msg = Float64()
        msg.data = vel

        # Publish message
        self.bkt_vel_msg = msg

    # Set the velocity of the bucket chain
    def set_conveyor_velocity(self, vel):

        # Limit value from 0.0 to 1.0
        vel = min(1.0, max(-1.0, vel))

        # Scale value to correct range
        vel = vel * self.max_vel_cnv

        # Create message
        msg = Float64()
        msg.data = vel

        # Publish message
        self.cnv_vel_msg = msg

    # --- PROCESS DIGGING COMMANDS --- #

    # Callback for dig commands - passes the command on to run_dig_operation
    # Only does anything if control_mode == "operation"
    def process_dig_command(self, dig_cmd_msg):

        # Motors off
        if dig_cmd_msg.data == "off":
            self.dig_op_off()

        # Zero the lead screw
        elif dig_cmd_msg.data == "zero_lsc":
            self.dig_op_zero_lsc()

        # Lower bucket chain and dig - not collecting material yet
        elif dig_cmd_msg.data == "dig":
            self.dig_op_dig()

        # Depositing material - conveyor only
        elif dig_cmd_msg.data == "deposit":
            self.dig_op_deposit()

        # Operation complete
        rospy.set_param("dig_operation_complete", True)

    # Set all dig motors to zero velocity
    def dig_op_off(self):

        self.set_bucket_chain_velocity(0.0)
        self.set_conveyor_velocity(0.0)
        self.set_lead_screw_velocity(0.0)

    # Zero the lead screw position by running it to the backwards limit switch
    def dig_op_zero_lsc(self):
        self.set_lead_screw_velocity(0.5)

        # Make sure everything but the hinge is off
        self.set_lead_screw_velocity(0.0)
        self.set_conveyor_velocity(0.0)
        self.set_bucket_chain_velocity(0.0)

        # Move the hinge to forward position
        self.set_hinge_position(0.0)

        # Wait for the hinge to get there
        self.continue_for_duration(3.0, 30)

        # Run the lead screw backwards until the limit switch is triggered
        self.set_lead_screw_velocity(-0.75)

        # Wait until we hit limit switch
        while not self.lsc_bkd_lim:
            self.publish_vel_pos()
            rospy.Rate(30).sleep()

    # Dig and collect material
    def dig_op_dig(self):

        # --- PREPARE TO DIG --- #

        # Make sure everything but the hinge is off
        self.set_lead_screw_velocity(0.0)
        self.set_conveyor_velocity(0.0)
        self.set_bucket_chain_velocity(0.0)

        # Command the hinge to the correct digging position
        self.set_hinge_position(0.0)

        # Wait for the hinge to get there
        self.continue_for_duration(3.0, 30)

        # --- DIG THROUGH BP1 - DON'T COLLECT --- #

        # Conveyor and bucket chain ON
        self.set_conveyor_velocity(0.2)
        self.set_bucket_chain_velocity(0.8)

        # Command lead screw to gravel depth
        self.set_lead_screw_position(self.gravel_depth, 0.75)

        # --- COLLECT GRAVEL --- #

        # Slow down conveyor
        self.set_conveyor_velocity(0.02)

        # Command lead screw to maximum depth
        self.set_lead_screw_position(self.max_depth, 0.75)

        # --- RETRACT DIGGING MECHANISM --- #

        # Conveyor and bucket chain off
        self.set_conveyor_velocity(0.0)
        self.set_bucket_chain_velocity(0.0)

        # Retract lead screw
        self.set_lead_screw_position(self.lsc_home_pos, 0.75)

        # Command the hinge to driving position
        self.set_hinge_position(1.0)

        # Wait for the hinge to get there
        self.continue_for_duration(3.0, 30)

    # Deposit material into the collection bin
    def dig_op_deposit(self):

        # Make sure everything but the hinge is off
        self.set_lead_screw_velocity(0.0)
        self.set_conveyor_velocity(0.0)
        self.set_bucket_chain_velocity(0.0)

        # Command the hinge out of the way of the conveyor
        self.set_hinge_position(0.0)

        # Wait for hinge to get there
        self.continue_for_duration(3.0, 30)

        # Turn on conveyor at high speed (and bucket chain ??)
        self.set_conveyor_velocity(0.9)

        # Wait for all material to be deposited
        self.set_conveyor_velocity(0.0)

        # Move hinge back to driving position
        self.set_hinge_position(1.0)

        # Wait for hinge to get there
        self.continue_for_duration(3.0, 30)


if __name__ == "__main__":

    # Initialize ROS node
    rospy.init_node("digging_controller")

    # Create a DigManager object
    dig_manager = DigManager()

    # Parameter to determine control mode
    control_mode = rospy.get_param("digging_control_mode", "pct_out")
    dig_manager.update_control_mode(control_mode)

    # Attach a subscriber to the dig command topic
    rospy.Subscriber("dig_cmd", String, dig_manager.process_dig_command)

    # Attach subscribers to individual motor speed topics
    rospy.Subscriber("dig_motor_pct_out/hinge", Float64, dig_manager.update_pct_out_hng)
    rospy.Subscriber("dig_motor_pct_out/lead_screw", Float64, dig_manager.update_pct_out_lsc)
    rospy.Subscriber("dig_motor_pct_out/bucket_chain", Float64, dig_manager.update_pct_out_bkt)
    rospy.Subscriber("dig_motor_pct_out/conveyor", Float64, dig_manager.update_pct_out_cnv)

    # Define loop rate - should be quite fast
    loop_rate = rospy.Rate(30)

    # Publish updates at loop rate
    while not rospy.is_shutdown():

        if dig_manager.control_mode == "pct_out":
            dig_manager.publish_pct_output()
        if dig_manager.control_mode == "vel_pos":
            dig_manager.publish_vel_pos()

        loop_rate.sleep()
