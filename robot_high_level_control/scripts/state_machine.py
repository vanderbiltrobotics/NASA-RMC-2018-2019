#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Pose, PointStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
import numpy as np
from math import radians

class HighLevelController:

    def __init__(self, starting_state="drive_to_p0"):

        # Initialize current state to the starting state
        self.current_state = starting_state

        # Publishers that need to be accessible by some state or function
        self.publishers = {
            "enable_pub_mot": rospy.Publisher("enable/motor_control", Bool, queue_size=0),
            "enable_pub_nav": rospy.Publisher("enable/navigation", Bool, queue_size=0),
            "enable_pub_loc": rospy.Publisher("enable/localization", Bool, queue_size=0),
            "enable_pub_obs": rospy.Publisher("enable/obstacles", Bool, queue_size=0),
            "enable_pub_top": rospy.Publisher("enable/teleop", Bool, queue_size=0),
            "drive_cmd": rospy.Publisher('drive_cmd', Twist, queue_size=0),
            "pub_goal_point": rospy.Publisher('goal_point', PointStamped, queue_size=0)
        }

        # Variables to use for
        self.state_vars = {
            "outbound": True,
            "P0": (2.0, 1.0),
            "P1": (1.5, 2.0),
            "P2": (1.5, 4.0),
            "P3": (1.5, 5.0),
            "drivability_map": None,
            "max_x_offset": 1.5,
            "P1_y": 2.5,
            "P2_y": 5.5,
            "pass_num": 1,
            "marker_detected": False
        }

        # Parameters to configure
        self.params = {
            "init_dig_y_loc": 6.0,
            "dig_y_loc_delta": 0.2,
            "localization_rotate_speed_deg": 1.0,
            "localization_rotate_angle_deg": 45.0
        }

    def process_state_changes(self):
        # Check for any control commands, loss of localization, etc.
        return

    def advance_state(self):

        # Handle navigation steps  (these are all straightforward)
        if self.current_state == "drive_to_p0":
            if self.state_vars["outbound"]:
                self.current_state = "drive_to_p1"
            else:
                self.current_state = "idle"
        elif self.current_state == "drive_to_p1":
            if self.state_vars["outbound"]:
                self.current_state = "drive_to_p2"
            else:
                self.current_state = "drive_to_p0"
        elif self.current_state == "drive_to_p2":
            if self.state_vars["outbound"]:
                self.current_state = "drive_to_p3"
            else:
                self.current_state = "drive_to_p1"
        elif self.current_state == "drive_to_p3":
            self.current_state = "drive_to_p2"

        return

    def run_states(self):

        # Continue running states until we're done
        while not rospy.is_shutdown():

            rospy.loginfo("advanced to state: " + self.current_state)

            if self.current_state == "idle":
                self.state_idle()
            elif self.current_state == "localizing":
                self.state_localizing()
            elif self.current_state == "drive_to_p0":
                self.state_drive_to_P0()
            elif self.current_state == "drive_to_p1":
                self.state_drive_to_P1()
            elif self.current_state == "drive_to_p2":
                self.state_drive_to_P2()
            elif self.current_state == "drive_to_p3":
                self.state_drive_to_P3()
            elif self.current_state == "mine_gravel":
                self.state_mine_gravel()
            elif self.current_state == "approach_bin":
                self.state_approach_bin()
            elif self.current_state == "deposit_gravel":
                self.state_deposit_gravel()
            elif self.current_state == "teleop_control":
                self.state_teleop_control()

        return

    # Configure the enable state for each group
    # Each input parameter should be a boolean
    def set_enables(self, motors=False, nav=False, localization=False, obstacles=False, teleop=False):

        # Create ROS bool messages
        mot_msg = Bool()
        nav_msg = Bool()
        loc_msg = Bool()
        obs_msg = Bool()
        top_msg = Bool()

        # Set data accordingly
        mot_msg.data = motors
        nav_msg.data = nav
        loc_msg.data = localization
        obs_msg.data = obstacles
        top_msg.data = teleop

        # Publish messages
        self.publishers["enable_pub_mot"].publish(mot_msg)
        self.publishers["enable_pub_nav"].publish(nav_msg)
        self.publishers["enable_pub_loc"].publish(loc_msg)
        self.publishers["enable_pub_obs"].publish(obs_msg)
        self.publishers["enable_pub_top"].publish(top_msg)

    # Configure P0 points
    def set_P0(self, x, y):

        self.state_vars["P0"] = [x, y]

    # Configure y values for P1 and P2
    def set_P1_P2_ybounds(self, y_min, y_max):

        self.state_vars["P1_y"] = y_min
        self.state_vars["P2_y"] = y_max

    # Set maximum offset from center that robot's target points can reach
    def set_max_x_offset(self, offset):
        self.state_vars["max_x_offset"] = offset

    # Checks the "goal_reached" command - resets the param if True
    def reached_goal(self, reset_if_true=False):

        # Check param
        val = rospy.get_param("goal_reached", default=False)

        # If goal reached, reset param to False
        if val and reset_if_true:
            rospy.set_param("goal_reached", False)

        # Return the value
        return val

    ## TODO DEFINE BEHAVIORS FOR EACH STATE ##

    # Idle state - robot does nothing, waits for further command
    def state_idle(self):

        # Disable everything except motor control nodes
        self.set_enables(motors=True)

        # Command motor speeds to zero
        self.publishers["drive_cmd"].publish(Twist())

        # Disable motor control nodes
        self.set_enables()

        # Set loop rate
        loop_rate = rospy.Rate(10)

        # Wait for further instruction
        while self.current_state == "idle" and not rospy.is_shutdown():

            # Sleep at loop rate
            loop_rate.sleep()

            # Process any commands we may have received
            self.process_state_changes()

        # Advance to next state
        self.advance_state()

        return

    # Localize only - sweep camera and rotate in place until markers located
    def state_localizing(self):

        # Enable motor and localization nodes
        self.set_enables(
            motors=True,
            localization=True,
        )

        # Record start time for current servo sweep
        start_time = rospy.Time.now()

        # Define loop rate
        loop_rate = rospy.Rate(10)
        rotation_loop_rate = rospy.Rate(20)

        # Define message for rotating the robot in place
        rotate_msg = Twist()
        rotate_msg.angular.z = radians(self.params["localization_rotate_speed_deg"])

        # Define duration of each rotation based on desired rotation angle
        rotate_duration = radians(self.params["localizaton_rotate_angle_deg"]) / rotate_msg.angular.z

        # Rotate robot in spurts until markers are located
        while self.current_state == "localizing" and not self.state_vars["marker_detected"]:

            # If 10 seconds have passed, rotate robot ~45 degrees
            if rospy.Time.now() - start_time > 10.0:

                # Rotate
                self.publishers["drive_cmd"].publish(rotate_msg)

                # Wait for the correct duration
                rotation_start = rospy.Time.now()

                while (rospy.Time.now() - rotation_start) < rotate_duration:
                    rotation_loop_rate.sleep()

                # Tell robot to stop
                self.publishers["drive_cmd"].publish(Twist())

            # Otherwise, wait and process control commands
            loop_rate.sleep()
            self.process_state_changes()

        # Advance state
        self.advance_state()

        return

    # Sends P0 as goal position and waits for robot to get there
    def state_drive_to_P0(self):

        # Make sure that all nodes are active
        self.set_enables(
            motors=True,
            nav=True,
            localization=True,
            obstacles=True
        )

        # Change the pure pursuit lookahead distance

        # Send P0 as a goal position
        point_msg = PointStamped()
        point_msg.header.frame_id = "world"
        point_msg.point.x = self.state_vars["P0"][0]
        point_msg.point.y = self.state_vars["P0"][1]
        self.publishers["pub_goal_point"].publish(point_msg)

        # Set loop rate
        loop_rate = rospy.Rate(5)

        # Wait for robot to reach goal pos or for other interrupt
        while self.current_state == "drive_to_p0" and not self.reached_goal(reset_if_true=True) and not rospy.is_shutdown():

            # Keep publishing in case we missed it
            self.publishers["pub_goal_point"].publish(point_msg)

            # Sleep at loop rate
            loop_rate.sleep()

            # Process any commands we may have received
            self.process_state_changes()

        # Advance to the next state
        self.advance_state()

        # Return
        return

    # Sends P1 as goal position (computes P1 & P2 if outbound), waits for robot to get there
    def state_drive_to_P1(self):

        # Make sure that all nodes are active
        self.set_enables(
            motors=True,
            nav=True,
            localization=True,
            obstacles=True
        )

        # Set loop rate
        loop_rate = rospy.Rate(10)

        # If outbound, compute P1 and P2
        if self.state_vars["outbound"]:

            # Make sure map has been received
            while self.state_vars["drivability_map"] is None:

                # Wait for map to be received
                loop_rate.sleep()

            rospy.loginfo("Map received!!!")

            pts = compute_obs_region_endpoints(
                self.state_vars["drivability_map"],
                self.state_vars["P1_y"],
                self.state_vars["P2_y"],
                self.state_vars["max_x_offset"]
            )

            self.state_vars["P1"] = pts[0]
            self.state_vars["P2"] = pts[1]

            rospy.loginfo(pts[0])
            rospy.loginfo(pts[1])

        # Change the pure pursuit lookahead distance


        # Send P1 as a goal position
        point_msg = PointStamped()

        point_msg.point.x = self.state_vars["P1"][0]
        point_msg.point.y = self.state_vars["P1"][1]
        self.publishers["pub_goal_point"].publish(point_msg)



        # Wait for robot to reach goal pos or for other interrupt
        while self.current_state == "drive_to_p1" and not self.reached_goal(reset_if_true=True) and not rospy.is_shutdown():

            # Keep publishing in case we missed it
            self.publishers["pub_goal_point"].publish(point_msg)

            # Sleep at loop rate
            loop_rate.sleep()

            # Process any commands we may have received
            self.process_state_changes()

        # Advance to the next state
        self.advance_state()

        # Return
        return

    # Sends P2 as goal position (computes P1 & P2 if inbound), waits for robot to get there
    def state_drive_to_P2(self):

        # Make sure that all nodes are active
        self.set_enables(
            motors=True,
            nav=True,
            localization=True,
            obstacles=True
        )

        # If inbound, compute P1 and P2
        if not self.state_vars["outbound"]:

            # Get P1 and P2
            pts = compute_obs_region_endpoints(
                self.state_vars["drivability_map"],
                self.state_vars["P1_y"],
                self.state_vars["P2_y"],
                self.state_vars["max_x_offset"]
            )

            # Update corresponding state vars
            self.state_vars["P1"] = pts[0]
            self.state_vars["P2"] = pts[1]

        # Change the pure pursuit lookahead distance

        # Send P2 as a goal position
        point_msg = PointStamped()
        point_msg.header.frame_id = "world"
        point_msg.point.x = self.state_vars["P2"][0]
        point_msg.point.y = self.state_vars["P2"][1]
        self.publishers["pub_goal_point"].publish(point_msg)

        # Set loop rate
        loop_rate = rospy.Rate(10)

        # Wait for robot to reach goal pos or for other interrupt
        while self.current_state == "drive_to_p2" and not self.reached_goal(reset_if_true=True) and not rospy.is_shutdown():

            # Keep publishing in case we missed it
            self.publishers["pub_goal_point"].publish(point_msg)

            # Sleep at loop rate
            loop_rate.sleep()

            # Process any commands we may have received
            self.process_state_changes()

        # Advance to the next state
        self.advance_state()

        # Return
        return

    # Computes and sends P3 as goal position, waits for robot to get there
    def state_drive_to_P3(self):

        # Compute P3
        p3_x = self.state_vars["P2"][0]
        p3_y = self.params["init_dig_y_loc"] - (self.params["dig_y_loc_delta"] * (self.state_vars["pass_num"] - 1))

        # Update P3 state var
        self.state_vars["P3"] = (p3_x, p3_y)

        # Change the pure pursuit lookahead distance

        # Send P3 as a goal position
        point_msg = PointStamped()
        point_msg.header.frame_id = "world"
        point_msg.point.x = p3_x
        point_msg.point.y = p3_y
        self.publishers["pub_goal_point"].publish(point_msg)

        # Set loop rate
        loop_rate = rospy.Rate(10)

        # Wait for robot to reach goal pos or for other interrupt
        while self.current_state == "drive_to_p3" and not self.reached_goal(reset_if_true=True) and not rospy.is_shutdown():

            # Keep publishing in case we missed it
            self.publishers["pub_goal_point"].publish(point_msg)

            # Sleep at loop rate
            loop_rate.sleep()

            # Process any commands we may have received
            self.process_state_changes()

        # No longer outbound
        self.state_vars["outbound"] = False

        # Advance to the next state
        self.advance_state()

        # Return
        return

    # Configures the robot for teleop control
    def state_teleop_control(self):

        # Configure appropriate enables
        self.set_enables(
            motors=True,
            localization=True,
            nav=False,
            obstacles=True,
            teleop=True
        )

        # Define loop rate
        loop_rate = rospy.Rate(10)

        # Wait until we exit teleop mode
        while self.current_state == "teleop_control":

            # Sleep at loop rate
            loop_rate.sleep()

            # Check for state updates
            self.process_state_changes()

        # We only leave this state if a command was recieved. No need to advance state
        return

    def state_mine_gravel(self):
        return

    def state_approach_bin(self):
        return

    def state_deposit_gravel(self):
        return

    # ------------------------------------- #
    # CALLBACK FUNCTIONS FOR UPDATING STATE #
    # ------------------------------------- #

    def update_map(self, new_map_msg):

        # Get new map, make sure dimensions are correct
        map_length = new_map_msg.info.height
        map_width = new_map_msg.info.width

        # Convert to numpy array and store
        self.state_vars["drivability_map"] = np.reshape(np.array(new_map_msg.data), (map_length, map_width))


# ----------------------------------------------- #
# HELPER FUNCTIONS FOR DETERMINING GOAL POSITIONS #
# ----------------------------------------------- #

# Check if input point is within the field boundaries
def is_in_field(driveability_map, pt):
    return True if ((pt >= 0).all() and pt[0] < driveability_map.shape[1] and pt[1] < driveability_map.shape[0]) \
        else False


# Checks if the path between two points is clear
# Drivability map is in cm
# Start / end pos are in meters
def is_clear_path(driveability_map, start_pos, end_pos):

    # Convert start and end positions to cm
    start_pos = [int(i * 100) for i in start_pos]
    end_pos = [int(i * 100) for i in end_pos]

    # Set step size to 1 so we don' miss any obstacles
    step_size = 1

    # calculating length of the straight line and the number of points
    dy = end_pos[1] - start_pos[1]
    dx = end_pos[0] - start_pos[0]
    path_length = np.sqrt(dx ** 2 + dy ** 2)
    matrix_len = int(path_length / step_size)

    # creating matrices for calculations
    points = [start_pos for i in range(matrix_len)]
    steps = [[step_size * dx / path_length, step_size * dy / path_length] for i in range(matrix_len)]
    nums = [[i + 1, i + 1] for i in range(matrix_len)]

    # Calculating the path
    path = np.add(points, np.multiply(steps, nums))

    # Prepend starting point
    path = np.insert(path, 0, start_pos, axis=0)

    # Check for obstacles
    for pt in path:

        # Check boundaries
        if not is_in_field(driveability_map, pt):
            return False

        if driveability_map[int(pt[1]), int(pt[0])] == 1:
            return False  # obstacle detected, can't cross here

    return True  # Safe to drive along this line


# driveability_map is in cm, y_min and y_max are in m, max_x_offset is in cm
def compute_obs_region_endpoints(driveability_map, y_min, y_max, max_x_offset):

    # Get field width
    field_width = driveability_map.shape[1]

    # Starting p1 and p2; Converted to cm
    # Must retain original points in case no easy path is found
    p1_orig = [field_width / 2.0 / 100, y_min]
    p2_orig = [field_width / 2.0 / 100, y_max]

    # Rounded p1 and p2; Converted to cm
    # In case occupancy grid is odd
    p1 = [field_width / 2.0 / 100.0, y_min]
    p2 = [field_width / 2.0 / 100.0, y_max]

    # Left and right p1 and p2
    p1_left = np.array([0, 0])
    p2_left = np.array([0, 0])
    p1_right = np.array([0, 0])
    p2_right = np.array([0, 0])

    # Initialize to first enter while loop
    offset = -0.05  # cm; Becomes 0 right away in loop
    clear_path_left = False  # Need two paths in case occupancy grid has odd width
    clear_path_right = False

    # Search until clear path is found or field boundaries are reached
    while not clear_path_left and not clear_path_right and offset <= max_x_offset + 0.05:
        # Increase offset
        offset += 0.05

        # Add offset in both x-directions to avoid obstacles
        p1_right = (p1 + np.array([offset, 0])).tolist()
        p1_left = (p1 - np.array([offset, 0])).tolist()
        p2_right = (p2 + np.array([offset, 0])).tolist()
        p2_left = (p2 - np.array([offset, 0])).tolist()

        # check if p1 and p2 line up in a clear path
        # Path is sent in rounded cm to be compatible with driveability_map
        clear_path_right = is_clear_path(driveability_map, p1_right, p2_right)
        clear_path_left = is_clear_path(driveability_map, p1_left, p2_left)

    # Return points in clear path, w/ preference to the right
    # Return starting points if no path found
    if clear_path_right:
        return [p1_right, p2_right]
    elif clear_path_left:
        return [p1_left, p2_left]
    else:
        return [p1_orig, p2_orig]


# Create the controller and run ROS node
if __name__ == "__main__":

    # Initialize ROS node
    rospy.init_node("high_level_controller")

    # Create state machine
    state_machine = HighLevelController()

    # Attach subscribers
    rospy.Subscriber("test_map", OccupancyGrid, state_machine.update_map)

    # Run state machine
    state_machine.run_states()