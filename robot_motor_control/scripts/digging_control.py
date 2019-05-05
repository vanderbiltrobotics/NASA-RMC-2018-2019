#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, String


class DigManager:

    def __init__(self):

        # Current dig mode - "pct_out", "vel_pos", "operation"
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

        # Max velocity parameters for various motors
        self.max_vel_lsc = 16000.0
        self.max_vel_bkt = 700.0
        self.max_pos_hng = -881.0    # Fully retracted
        self.min_pos_hng = -725.0    # Digging point
        self.max_pos_lsc = 10000000
        self.min_pos_lsc = 0.0
        self.cnv_vel_dig = 100.0
        self.cnv_vel_collect = 10.0
        self.cnv_vel_deposit = 450.0

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

    # these callbacks will only do something if control mode is "teleop"

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

    # Moves hinge to a specific angle
    def set_hinge_position(self):
        return

    # Send a value from 0 to 1
    def set_lead_screw_position(self, pos, vel):


        pass

    def set_lead_screw_velocity(self, vel):

        # Limit value from 0.0 to 1.0
        vel = min(1.0, max(-1.0, vel))

        # Scale value to correct range
        vel = vel * self.max_vel_lsc

        # Create message
        msg = Float64()
        msg.data = vel

        # Publish message
        if self.control_mode == "vel_pos":
            self.publishers["pub_vel_lsc"].publish(msg)

    def set_bucket_chain_velocity(self, vel):

        # Limit value from 0.0 to 1.0
        vel = min(1.0, max(-1.0, vel))

        # Scale value to correct range
        vel = vel * self.max_vel_bkt

        # Create message
        msg = Float64()
        msg.data = vel

        # Publish message
        if self.control_mode == "vel_pos":
            self.publishers["pub_vel_bkt"].publish(msg)


    def set_conveyor_velocity(self):
        # Read in the desired bucket chain velocity
        return

    # --- PROCESS DIGGING COMMANDS --- #

    # Callback for dig commands - passes the command on to run_dig_operation
    # Only does anything if control_mode == "operation"
    def process_dig_command(self, dig_cmd_msg):

        # Motors off
        if dig_cmd_msg.data == "off":
            self.dig_op_off()

        # Raise hinge angle (make more vertical)
        elif dig_cmd_msg.data == "raise_hinge":
            self.dig_op_raise_hinge()

        # Lower bucket chain and dig - not collecting material yet
        elif dig_cmd_msg.data == "dig":
            self.dig_op_dig()

        # Raise bucket chain, don't dig
        elif dig_cmd_msg.data == "raise_bucket_chain":
            self.dig_op_raise_bucket_chain()

        # Lower hinge angle (make less vertical)
        elif dig_cmd_msg.data == "lower_hinge":
            self.dig_op_lower_hinge()

        # Depositing material - conveyor only
        elif dig_cmd_msg.data == "deposit_material":
            self.dig_op_deposit_material()


    # Motors off
    def dig_op_off(self):
        self.update_pct_output()

    def dig_op_zero_lsc(self):



    # Raise hinge angle (make more vertical)
    def dig_op_raise_hinge(self):

        ##TODO switch to hinge_to_dig_position()
        # hinge_to_dig_position()

        # Run hinge motor until correect upright angle achieved
        start_time = rospy.Time.now()

        # Turn hinge motor on
        self.update_pct_output(
            hng=0.2
        )

        # Define loop rate
        loop_rate = rospy.Rate(10)

        # Spin motors for the specified duration
        while rospy.Time.now() - start_time < 3.0:
            loop_rate.sleep()

        # Turn motors off
        self.update_pct_output()

    # Lower bucket chain and dig - not collecting material yet
    def dig_op_dig(self):

        # Run bucket chain motor and lead screw motor
        start_time = rospy.Time.now()

        # Define loop rate
        loop_rate = rospy.Rate(10)

        # --- LOWER BUCKET CHAIN TO GRAVEL DEPTH --- #
        # Use set_bucket_chain_velocity()

        # Turn bucket chain motor, lead screw motor, conveyor motors on
        self.update_pct_output(
            lsc=0.2,
            bkt=0.2,
            cnv=0.2
        )

        # Wait until gravel depth reached
        while rospy.Time.now() - start_time < 3.0:
            loop_rate.sleep()

        # --- COLLECT MATERIAL --- #

        collection_steps = 10
        pulse_conveyor_time = 1.0
        pulse_collect_time = 2.0

        # Advance conveyor in 10 steps
        for i in range(collection_steps):

            # Turn conveyor on
            self.update_pct_output(
                lsc=0.1,
                bkt=0.2,
                cnv=0.2
            )

            # Wait for specified pulse duration
            pulse_start = rospy.Time.now()

            while rospy.Time.now() - pulse_start < pulse_conveyor_time:
                loop_rate.sleep()

            # Turn conveyor off
            self.update_pct_output(
                lsc=0.1,
                bkt=0.2
            )

            # Collect material for specified collect duration
            pulse_start = rospy.Time.now()

            while rospy.Time.now() - pulse_start < pulse_collect_time:
                loop_rate.sleep()

            # Turn motors off
            self.update_pct_output()

        # Turn motors off
        self.update_pct_output()

    # Raise bucket chain, don't dig
    def dig_op_raise_bucket_chain(self):

        # Run bucket chain motor and lead screw motor
        start_time = rospy.Time.now()

        # Define loop rate
        loop_rate = rospy.Rate(10)

        # --- LOWER BUCKET CHAIN TO GRAVEL DEPTH --- #
        # Use set_bucket_chain_velocity

        # Retract bucket chain by running lead screw motor backwards
        self.update_pct_output(
            lsc=-0.2
        )

        # Wait until we've reached retracted position
        while rospy.Time.now() - start_time < 3.0:
            loop_rate.sleep()

        # Turn motors off
        self.update_pct_output()

    # Lower hinge angle (make less vertical)
    def dig_op_lower_hinge(self):

        # Run hinge motor until correect upright angle achieved
        start_time = rospy.Time.now()

        # Turn hinge motor on
        self.update_pct_output(
            hng=-0.2
        )

        # Define loop rate
        loop_rate = rospy.Rate(10)

        # Spin motors for the specified duration
        while rospy.Time.now() - start_time < 3.0:
            loop_rate.sleep()

        # Turn motors off
        self.update_pct_output()

    # Depositing material - conveyor only
    def dig_op_deposit_material(self):

        # Run conveyor for 5 seconds to empty all material
        start_time = rospy.Time.now()

        # Turn motors on
        self.update_pct_output(
            cnv=0.2
        )

        # Define loop rate
        loop_rate = rospy.Rate(10)

        # Spin motors for the specified duration
        while rospy.Time.now() - start_time < 5.0:

            loop_rate.sleep()

        # Turn motors off
        self.update_pct_output()



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
            pass

        loop_rate.sleep()


