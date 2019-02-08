#!/usr/bin/env python

# --------------------------------------------------------------------------------------- #
#
# DRIVE MOTOR CONTROLLER NODE
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
from std_msgs.msg import Float32


class DriveController:

    # Constructor
    def __init__(self, wheel_sep, wheel_rad, in_max_lin_vel, in_max_ang_vel,
                 out_max_lin_vel, out_max_ang_vel, out_format):

        # Store parameters
        self.wheel_separation = wheel_sep
        self.wheel_radius = wheel_rad
        self.in_max_lin_vel = in_max_lin_vel
        self.in_max_ang_vel = in_max_ang_vel
        self.out_max_lin_vel = out_max_lin_vel
        self.out_max_ang_vel = out_max_ang_vel
        self.out_format = out_format

        # Initialize publishers
        self.speed_pub_fl = rospy.Publisher('drive/motor_speeds/fl', Float32, queue_size=1)
        self.speed_pub_bl = rospy.Publisher('drive/motor_speeds/bl', Float32, queue_size=1)
        self.speed_pub_fr = rospy.Publisher('drive/motor_speeds/fr', Float32, queue_size=1)
        self.speed_pub_br = rospy.Publisher('drive/motor_speeds/br', Float32, queue_size=1)

        # Initialize subscriber
        self.cmd_sub = rospy.Subscriber('drive_cmd', Twist, self.process_drive_cmd)

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

    # Callback function for new drive commands
    def process_drive_cmd(self, data):

        # Extract the relevant values from the incoming message
        linear = data.linear.x
        angular = data.angular.x

        # Remap the velocities to the desired output range
        linear = self.remap_lin_vel(linear)
        angular = self.remap_ang_vel(angular)

        # Compute linear and angular velocity components
        linear_component = linear / self.wheel_radius
        angular_component = (angular * self.wheel_separation) / self.wheel_radius

        # Compute speed for left and right motor (for ang. vel., clockwise = negative, cc = positive)
        right_speeds = linear_component - angular_component
        left_speeds = linear_component + angular_component

        # If out_format = pwm, convert velocities back to pwm
        if self.out_format == "pwm":
            left_speeds, right_speeds = self.wheel_vels_to_pwms(left_speeds, right_speeds)

        # Create messages for publishing motor speeds
        motor_speed_fl = Float32()
        motor_speed_bl = Float32()
        motor_speed_fr = Float32()
        motor_speed_br = Float32()

        # Set message values
        motor_speed_fl.data = left_speeds
        motor_speed_bl.data = left_speeds
        motor_speed_fr.data = right_speeds
        motor_speed_br.data = right_speeds

        # Publish messages
        self.speed_pub_fl.publish(motor_speed_fl)
        self.speed_pub_bl.publish(motor_speed_bl)
        self.speed_pub_fr.publish(motor_speed_fr)
        self.speed_pub_br.publish(motor_speed_br)


# Run the node
if __name__ == '__main__':

    # Initialize as ROS node
    rospy.init_node('drive_motor_controller')

    # Read values from server
    wheel_sep = rospy.get_param("robot/wheel_separation")
    wheel_rad = rospy.get_param("robot/wheel_radius")
    in_max_lin_vel = rospy.get_param("drive/max_in_lin_vel")
    in_max_ang_vel = rospy.get_param("drive/max_in_ang_vel")
    out_max_lin_vel = rospy.get_param("drive/max_out_lin_vel")
    out_max_ang_vel = rospy.get_param("drive/max_out_ang_vel")
    output_format = rospy.get_param("drive/exp_motor_cmd_fmt")

    # Create a DriveController object
    controller = DriveController(wheel_rad, wheel_rad, in_max_lin_vel, in_max_ang_vel,
                                 out_max_lin_vel, out_max_ang_vel, output_format)

    # Ready to go
    rospy.loginfo("Drive Motor Controller initialized...")

    # Loop continuously
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        pass
