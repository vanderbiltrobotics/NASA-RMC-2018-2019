#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, String


class DigManager:

    def __init__(self):

        # Current dig mode - can be "teleop" or "auto"
        self.dig_mode = "teleop"

        # Publishers for each dig motor
        self.publishers = {
            "pub_hng": rospy.Publisher("hinge_motor/set_percent_output", Float64, queue_size=0),
            "pub_lsc": rospy.Publisher("lead_screw_motor/set_percent_output", Float64, queue_size=0),
            "pub_bkt": rospy.Publisher("bucket_motor/set_percent_output", Float64, queue_size=0),
            "pub_cnv": rospy.Publisher("conveyor_motor/set_percent_output", Float64, queue_size=0)
        }

        # Messages storing current speeds for each motor
        self.hng_msg = Float64()
        self.lsc_msg = Float64()
        self.bkt_msg = Float64()
        self.cnv_msg = Float64()

    # Callback for dig mode changes
    def update_dig_mode(self, msg):
        self.dig_mode = msg.data

    # Update current speed setting for each motor
    # Mask can be applied to only set speeds for specific motor(s)
    def update_speeds(self, hng=0.0, lsc=0.0, bkt=0.0, cnv=0.0, update_mask=(1,1,1,1)):

        # Set message speeds
        if update_mask[0] == 1:
            self.hng_msg.data = hng
        if update_mask[1] == 1:
            self.lsc_msg.data = lsc
        if update_mask[2] == 1:
            self.bkt_msg.data = bkt
        if update_mask[3] == 1:
            self.cnv_msg.data = cnv

    # Publish messages to set speed of each digging motor
    def publish_speeds(self):

        # Publish messages
        self.publishers["pub_hng"].publish(self.hng_msg)
        self.publishers["pub_lsc"].publish(self.lsc_msg)
        self.publishers["pub_bkt"].publish(self.bkt_msg)
        self.publishers["pub_cnv"].publish(self.cnv_msg)

    # --- UPDATE INDIVIDUAL MOTOR SPEEDS --- #

    # these callbacks will only do something if control mode is "teleop"

    def update_speed_hng(self, new_speed):
        if self.dig_mode == "teleop":
            self.update_speeds(hng=new_speed.data, update_mask=(1, 0, 0, 0))

    def update_speed_lsc(self, new_speed):
        if self.dig_mode == "teleop":
            self.update_speeds(lsc=new_speed.data, update_mask=(0, 1, 0, 0))

    def update_speed_bkt(self, new_speed):
        if self.dig_mode == "teleop":
            self.update_speeds(bkt=new_speed.data, update_mask=(0, 0, 1, 0))

    def update_speed_cnv(self, new_speed):
        if self.dig_mode == "teleop":
            self.update_speeds(cnv=new_speed.data, update_mask=(0, 0, 0, 1))

    # Callback for dig commands - passes the command on to run_dig_operation
    # Only does anything if dig_mode == "auto"
    def process_dig_command(self, dig_cmd_msg):
        if self.dig_mode == "auto":
            self.run_dig_operation(dig_cmd_msg.data)


    # Moves hinge to a specific angle
    def set_hinge_angle(self):
        return

    def move_to_digging_angle(self):
        # Read in the desired digging angle from the parameter server
        # Use set_hinge_angle function to move to desired location
        return

    def set_lead_screw_position(self):
        # Read in the desired lead screw position
        return

    def set_lead_screw_velocity(self):
        # Read in the desired lead screw velocity
        return

    def set_bucket_chain_velocity(self):
        # Read in the desired bucket chain velocity
        return


    # Motors off
    def dig_op_off(self):
        self.update_speeds()

    # Raise hinge angle (make more vertical)
    def dig_op_raise_hinge(self):

        ##TODO switch to hinge_to_dig_position()
        # hinge_to_dig_position()

        # Run hinge motor until correect upright angle achieved
        start_time = rospy.Time.now()

        # Turn hinge motor on
        self.update_speeds(
            hng=0.2
        )

        # Define loop rate
        loop_rate = rospy.Rate(10)

        # Spin motors for the specified duration
        while rospy.Time.now() - start_time < 3.0:
            loop_rate.sleep()

        # Turn motors off
        self.update_speeds()

    # Lower bucket chain and dig - not collecting material yet
    def dig_op_dig(self):

        # Run bucket chain motor and lead screw motor
        start_time = rospy.Time.now()

        # Define loop rate
        loop_rate = rospy.Rate(10)

        # --- LOWER BUCKET CHAIN TO GRAVEL DEPTH --- #
        # Use set_bucket_chain_velocity()

        # Turn bucket chain motor, lead screw motor, conveyor motors on
        self.update_speeds(
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
            self.update_speeds(
                lsc=0.1,
                bkt=0.2,
                cnv=0.2
            )

            # Wait for specified pulse duration
            pulse_start = rospy.Time.now()

            while rospy.Time.now() - pulse_start < pulse_conveyor_time:
                loop_rate.sleep()

            # Turn conveyor off
            self.update_speeds(
                lsc=0.1,
                bkt=0.2
            )

            # Collect material for specified collect duration
            pulse_start = rospy.Time.now()

            while rospy.Time.now() - pulse_start < pulse_collect_time:
                loop_rate.sleep()

            # Turn motors off
            self.update_speeds()

        # Turn motors off
        self.update_speeds()

    # Raise bucket chain, don't dig
    def dig_op_raise_bucket_chain(self):

        # Run bucket chain motor and lead screw motor
        start_time = rospy.Time.now()

        # Define loop rate
        loop_rate = rospy.Rate(10)

        # --- LOWER BUCKET CHAIN TO GRAVEL DEPTH --- #
        # Use set_bucket_chain_velocity

        # Retract bucket chain by running lead screw motor backwards
        self.update_speeds(
            lsc=-0.2
        )

        # Wait until we've reached retracted position
        while rospy.Time.now() - start_time < 3.0:
            loop_rate.sleep()

        # Turn motors off
        self.update_speeds()

    # Lower hinge angle (make less vertical)
    def dig_op_lower_hinge(self):

        # Run hinge motor until correect upright angle achieved
        start_time = rospy.Time.now()

        # Turn hinge motor on
        self.update_speeds(
            hng=-0.2
        )

        # Define loop rate
        loop_rate = rospy.Rate(10)

        # Spin motors for the specified duration
        while rospy.Time.now() - start_time < 3.0:
            loop_rate.sleep()

        # Turn motors off
        self.update_speeds()

    # Depositing material - conveyor only
    def dig_op_deposit_material(self):

        # Run conveyor for 5 seconds to empty all material
        start_time = rospy.Time.now()

        # Turn motors on
        self.update_speeds(
            cnv=0.2
        )

        # Define loop rate
        loop_rate = rospy.Rate(10)

        # Spin motors for the specified duration
        while rospy.Time.now() - start_time < 5.0:

            loop_rate.sleep()

        # Turn motors off
        self.update_speeds()

    # Runs one of the primary digging operations
    def run_dig_operation(self, operation):


        # Motors off
        if self.dig_mode == "off":
            self.dig_op_off()

        # Raise hinge angle (make more vertical)
        elif self.dig_mode == "raise_hinge":
            self.dig_op_raise_hinge()

        # Lower bucket chain and dig - not collecting material yet
        elif self.dig_mode == "dig":
            self.dig_op_dig()

        # Raise bucket chain, don't dig
        elif self.dig_mode == "raise_bucket_chain":
            self.dig_op_raise_bucket_chain()

        # Lower hinge angle (make less vertical)
        elif self.dig_mode == "lower_hinge":
            self.dig_op_lower_hinge()

        # Depositing material - conveyor only
        elif self.dig_mode == "deposit_material":
            self.dig_op_deposit_material()

        else:
            # Error or something
            return


if __name__ == "__main__":

    # Initialize ROS node
    rospy.init_node("digging_controller")

    # Create a DigManager object
    dig_manager = DigManager()

    # Attach a subscriber to the dig mode topic (teleop vs. auto)
    rospy.Subscriber("dig_mode", String, dig_manager.update_dig_mode)

    # Attach a subscriber to the dig command topic
    rospy.Subscriber("dig_cmd", String, dig_manager.process_dig_command)

    # Attach subscribers to individual motor speed topics
    rospy.Subscriber("dig_motor_speeds/hinge", Float64, dig_manager.update_speed_hng)
    rospy.Subscriber("dig_motor_speeds/lead_screw", Float64, dig_manager.update_speed_lsc)
    rospy.Subscriber("dig_motor_speeds/bucket_chain", Float64, dig_manager.update_speed_bkt)
    rospy.Subscriber("dig_motor_speeds/conveyor", Float64, dig_manager.update_speed_cnv)

    # Define loop rate - should be quite fast
    loop_rate = rospy.Rate(30)

    # Publish updates at loop rate
    while not rospy.is_shutdown():

        dig_manager.publish_speeds()
        loop_rate.sleep()


