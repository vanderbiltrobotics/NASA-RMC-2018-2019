import rospy
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Bool
import numpy as np
from math import radians

class HighLevelController:

    def __init__(self, starting_state="idle"):

        # Initialize current state to the starting state
        self.current_state = starting_state

        # Publishers that need to be accessible by some state or function
        self.publishers = {
            "enable_pub_mot": rospy.Publisher("enable/motor_control", Bool, queue_size=1),
            "enable_pub_nav": rospy.Publisher("enable/navigation", Bool, queue_size=1),
            "enable_pub_loc": rospy.Publisher("enable/localization", Bool, queue_size=1),
            "enable_pub_obs": rospy.Publisher("enable/obstacles", Bool, queue_size=1),
            "enable_pub_top": rospy.Publisher("enable/teleop", Bool, queue_size=1),
            "drive_cmd": rospy.Publisher('drive_cmd', Twist, queue_size=1),
            "pub_goal_pose": rospy.Publisher('goal_pos', Pose, queue_size=1)
        }

        # Variables to use for
        self.state_vars = {
            "outbound": True,
            "P0": (1.5, 1.2),
            "P1": (1.5, 2.0),
            "P2": (1.5, 4.0),
            "P3": (1.5, 5.0),
            "drivability_map": np.zeros(shape=(500, 500)),
            "max_x_offset": 1.5,
            "P1_y": 2.0,
            "P2_y": 4.0,
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
        # Considering the curren state and other info (such as elapsed time), advance to the correct next state
        return

    def run_states(self):

        # Continue running states until we're done
        while not rospy.is_shutdown():

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
        while self.current_state == "idle":

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
        pose_msg = Pose()
        pose_msg.position.x = self.state_vars["P0"][0]
        pose_msg.position.y = self.state_vars["P0"][1]
        self.publishers["pub_goal_pose"].publish(pose_msg)

        # Set loop rate
        loop_rate = rospy.Rate(10)

        # Wait for robot to reach goal pos or for other interrupt
        while self.current_state == "drive_to_p0" and self.reached_goal(reset_if_true=True):

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

        # If outbound, compute P1 and P2
        if self.state_vars["outbound"]:

            pts = compute_obs_region_endpoints(
                self.state_vars["drivability_map"],
                self.state_vars["P1_y"],
                self.state_vars["P2_y"],
                self.state_vars["max_x_offset"]
            )

            self.state_vars["P1"] = pts[0]
            self.state_vars["P2"] = pts[1]

        # Change the pure pursuit lookahead distance


        # Send P1 as a goal position
        pose_msg = Pose()
        pose_msg.position.x = self.state_vars["P1"][0]
        pose_msg.position.y = self.state_vars["P1"][1]
        self.publishers["pub_goal_pose"].publish(pose_msg)

        # Set loop rate
        loop_rate = rospy.Rate(10)

        # Wait for robot to reach goal pos or for other interrupt
        while self.current_state == "drive_to_p1" and self.reached_goal(reset_if_true=True):

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
        if self.state_vars["outbound"]:

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
        pose_msg = Pose()
        pose_msg.position.x = self.state_vars["P2"][0]
        pose_msg.position.y = self.state_vars["P2"][1]
        self.publishers["pub_goal_pose"].publish(pose_msg)

        # Set loop rate
        loop_rate = rospy.Rate(10)

        # Wait for robot to reach goal pos or for other interrupt
        while self.current_state == "drive_to_p2" and self.reached_goal(reset_if_true=True):

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
        p3_y = self.params["init_dig_y_loc"] - (self.params["dig_y_loc_delta"] * self.state_vars["pass_num"])

        # Update P3 state var
        self.state_vars["P3"] = (p3_x, p3_y)

        # Change the pure pursuit lookahead distance

        # Send P3 as a goal position
        pose_msg = Pose()
        pose_msg.position.x = p3_x
        pose_msg.position.y = p3_y
        self.publishers["pub_goal_pose"].publish(pose_msg)

        # Set loop rate
        loop_rate = rospy.Rate(10)

        # Wait for robot to reach goal pos or for other interrupt
        while self.current_state == "drive_to_p3" and self.reached_goal(reset_if_true=True):

            # Sleep at loop rate
            loop_rate.sleep()

            # Process any commands we may have received
            self.process_state_changes()

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


# ----------------------------------------------- #
# HELPER FUNCTIONS FOR DETERMINING GOAL POSITIONS #
# ----------------------------------------------- #

# Check if input point is within the field boundaries
def is_in_field(driveability_map, pt):
    return True if ((pt >= 0).all() and pt[0] < driveability_map.shape[0] and pt[1] < driveability_map.shape[1]) \
        else False


# Check occupancy grid for obstacles along calculated path
# Takes path as type int to be compatible with driveability_map
def is_clear_path(driveability_map, path):

    # Check for obstacles
    for pt in path:

        # Check boundaries
        if not is_in_field(driveability_map, pt):
            return False

        occupied = driveability_map[pt[0], pt[1]]
        if occupied == 1:
            return False  # obstacle detected, can't cross here

    return True  # Safe to drive along this line


# Compute a simple path along the hypotenuse between the start and end points
# Contains start point in path
def compute_easy_path(start_pos, end_pos, step_size):

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
    return path


# driveability_map is in cm, y_min and y_max are in m, max_x_offset is in cm
def compute_obs_region_endpoints(driveability_map, y_min, y_max, max_x_offset):

    # Get field width
    field_width = driveability_map.shape[1]

    # Starting p1 and p2; Converted to cm
    # Must retain original points in case no easy path is found
    p1_orig = (y_min * 100.0, field_width / 2.0)
    p2_orig = (y_max * 100.0, field_width / 2.0)

    # Rounded p1 and p2; Converted to cm
    # In case occupancy grid is odd
    p1 = (y_min * 100.0, np.rint(field_width / 2.0))
    p2 = (y_max * 100.0, np.rint(field_width / 2.0))

    # Left and right p1 and p2
    p1_left = (0, 0)
    p2_left = (0, 0)
    p1_right = (0, 0)
    p2_right = (0, 0)

    # Initialize to first enter while loop
    left_offset = -1  # cm; Becomes 0 right away in loop
    right_offset = -1
    clear_path_left = False  # Need two paths in case occupancy grid has odd width
    clear_path_right = False

    # Search until clear path is found or field boundaries are reached
    while not clear_path_left and not clear_path_right and left_offset <= max_x_offset \
            and right_offset <= max_x_offset+1:
        # Increase offset
        right_offset += 1
        left_offset += 1

        # Add offset in both x-directions to avoid obstacles
        p1_right = (p1[0], p1[1] + right_offset)
        p1_left = (p1[0], p1[1] - left_offset)
        p2_right = (p2[0], p2[1] + right_offset)
        p2_left = (p2[0], p2[1] - left_offset)

        # Computes easy path in both directions
        # Step size is the same as for driveability_map
        # Points and step size in cm
        path_right = compute_easy_path(p1_right, p2_right, step_size=1)
        path_left = compute_easy_path(p1_left, p2_left, step_size=1)

        # check if p1 and p2 line up in a clear path
        # Path is sent in rounded cm to be compatible with driveability_map
        clear_path_right = is_clear_path(driveability_map, (path_right+0.5).astype(int))
        clear_path_left = is_clear_path(driveability_map, (path_left+0.5).astype(int))

    # Return points in clear path, w/ preference to the right
    # Return starting points if no path found
    if clear_path_right:
        return [p1_right, p2_right]
    elif clear_path_left:
        return [p1_left, p2_left]
    else:
        return [p1_orig, p2_orig]