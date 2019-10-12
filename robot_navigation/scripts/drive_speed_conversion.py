#!/usr/bin/env python

# --------------------------------------------------------------------------------------- #
#
# DRIVE SPEED CONVERSION NODE
#
# This node receives commands specifying the desired linear and angular velocity
# for the full robot. It determines the speeds that each wheel needs to move at
# for the robot to move at the commanded velocities. The output commands can be
# formatted either as wheel velocities (in rad / sec) or as pwm values (from -1.0 to 1.0)
#
# --------------------------------------------------------------------------------------- #

# Import ROS packages
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

# Import other required packages
from collections import deque

from math import pi


class DriveController:

    # Constructor
    def __init__(self, wheel_sep, wheel_rad, in_max_lin_vel, in_max_ang_vel,
                 out_max_lin_vel, out_max_ang_vel, out_format, queue_size):

        # Store parameters
        self.wheel_separation = wheel_sep
        self.wheel_radius = wheel_rad
        self.in_max_lin_vel = in_max_lin_vel
        self.in_max_ang_vel = in_max_ang_vel
        self.out_max_lin_vel = out_max_lin_vel
        self.out_max_ang_vel = out_max_ang_vel
        self.out_format = out_format
        self.max_queue_size = queue_size

        # Store float message for each motor speed
        self.speed_bl = Float64()
        self.speed_br = Float64()
        self.speed_fl = Float64()
        self.speed_fr = Float64()

        # Initialize queue list
        self.lin_queue = deque([0] * queue_size)
        self.ang_queue = deque([0] * queue_size)

        # Initialize publishers
        topic_str = "set_percent_output" if self.out_format == "pwm" else "set_velocity"
        self.speed_pub_fl = rospy.Publisher('front_left/' + topic_str, Float64, queue_size=1)
        self.speed_pub_bl = rospy.Publisher('back_left/' + topic_str, Float64, queue_size=1)
        self.speed_pub_fr = rospy.Publisher('front_right/' + topic_str, Float64, queue_size=1)
        self.speed_pub_br = rospy.Publisher('back_right/' + topic_str, Float64, queue_size=1)

        # Initialize subscriber
        self.cmd_sub = rospy.Subscriber('drive_cmd', Twist, self.process_drive_cmd)

        # Value to check if command was received this time through the cycle
        self.received_command = False

    # Remap linear velocity from input range to output range
    def remap_lin_vel(self, vel):
        return (vel / self.in_max_lin_vel) * self.out_max_lin_vel

    # Remap angular velocity from input range to output range
    def remap_ang_vel(self, vel):
        return (vel / self.in_max_ang_vel) * self.out_max_ang_vel

    # Maps from wheel velocities (in rad / sec) to pwm values - if either of the values would exceed
    # a pwm of 1.0, both values are scaled down appropriately such that the same ratio is maintained
    # but neither pwm exceeds 1.0. NOTE: the actual speed that results from this PWM setting
    # will NOT be the same as the input velocity values - this is simply a way of producing usable
    # pwm values from velocity commands
    def wheel_vels_to_pwms(self, left_vel, right_vel):

        # Scale each value by max lin. vel. output (converted to rad / sec)
        left_pwm = left_vel / (self.out_max_lin_vel / self.wheel_radius)
        right_pwm = right_vel / (self.out_max_lin_vel / self.wheel_radius)

        # If either exceeds 1.0, scale the values so max = 1.0
        max_val = max(left_pwm, right_pwm)
        if max_val > 1.0:
            left_pwm = left_pwm / max_val
            right_pwm = right_pwm / max_val

        # Return the pair of scaled values as a tuple
        return left_pwm, right_pwm

    # Calculate average velocities in queue of inputs
    def avg_queue_vel(self, data):

        # Get linear, angular components of Twist message
        linear = data.linear.x
        angular = data.angular.z

        # Update linear queue
        self.lin_queue.append(linear)
        self.lin_queue.popleft()

        # Update angular queue
        self.ang_queue.append(angular)
        self.ang_queue.popleft()

        # Find the average value in each queue
        lin_avg = sum(self.lin_queue) / len(self.lin_queue)
        ang_avg = sum(self.ang_queue) / len(self.ang_queue)

        return lin_avg, ang_avg

    # Callback function for new drive commands
    def process_drive_cmd(self, data):

        # Indicate that we received a command
        self.received_command = True

        # Extract the relevant values from the incoming message
        linear, angular = self.avg_queue_vel(data)

        # Remap the velocities to the desired output range
        linear = self.remap_lin_vel(linear)
        angular = self.remap_ang_vel(angular)

        # Value to scale units to what talons are expecting
        scale = 2048.0 / (10.0 * pi)

        # Compute linear and angular velocity components
        linear_component = (linear / self.wheel_radius) * scale
        angular_component = ((angular * self.wheel_separation) / self.wheel_radius) * scale

        # Compute speed for left and right motor (for ang. vel., clockwise = negative, cc = positive)
        right_speeds = linear_component + angular_component
        left_speeds = linear_component - angular_component

        # If out_format = pwm, convert velocities back to pwm
        if self.out_format == "pwm":
            left_speeds, right_speeds = self.wheel_vels_to_pwms(left_speeds, right_speeds)

        # Create messages for publishing motor speeds
        motor_speed_fl = Float64()
        motor_speed_bl = Float64()
        motor_speed_fr = Float64()
        motor_speed_br = Float64()

        # Set message values
        motor_speed_fl.data = left_speeds
        motor_speed_bl.data = left_speeds
        motor_speed_fr.data = right_speeds
        motor_speed_br.data = right_speeds

        # Publish messages
        self.speed_bl = motor_speed_bl
        self.speed_br = motor_speed_br
        self.speed_fl = motor_speed_fl
        self.speed_fr = motor_speed_fr


    def publish_speeds(self):

        self.speed_pub_fl.publish(self.speed_fl)
        self.speed_pub_bl.publish(self.speed_bl)
        self.speed_pub_fr.publish(self.speed_fr)
        self.speed_pub_br.publish(self.speed_br)


# Run the node
if __name__ == '__main__':

    # Initialize as ROS node
    rospy.init_node('drive_motor_controller')

    # Read values from server
    wheel_sep = rospy.get_param("wheel_separation")
    wheel_rad = rospy.get_param("wheel_radius")
    in_max_lin_vel = rospy.get_param("max_in_lin_vel")
    in_max_ang_vel = rospy.get_param("max_in_ang_vel")
    out_max_lin_vel = rospy.get_param("max_out_lin_vel")
    out_max_ang_vel = rospy.get_param("max_out_ang_vel")
    output_format = rospy.get_param("output_mode")
    queue_size = rospy.get_param("queue_size")

    # Create a DriveController object
    controller = DriveController(wheel_rad, wheel_rad, in_max_lin_vel, in_max_ang_vel,
                                 out_max_lin_vel, out_max_ang_vel, output_format, queue_size)

    # Ready to go
    rospy.loginfo("Drive Motor Controller initialized...")

    # Define loop rate - should be quite fast
    loop_rate = rospy.Rate(30)

    # Publish updates at loop rate
    while not rospy.is_shutdown():

        # Check if we received a command
        if not controller.received_command:
            controller.process_drive_cmd(Twist())

        # Reset received_command tracker
        controller.received_command = False

        # Pulish message and sleep
        controller.publish_speeds()
        loop_rate.sleep()
