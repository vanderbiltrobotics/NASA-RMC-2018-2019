import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class HighLevelController:

    def __init__(self, starting_state = "idle"):

        # Initialize current state to the starting state
        self.current_state = starting_state

        self.p0 = [0,0]
        self.p1 = [0,0]
        self.p2 = [3,3]
        self.p3 = [0,0]

        self.publishers = {
            "drive_cmd" : rospy.Publisher('drive_cmd', Twist, queue_size=1),
            "enable_pub_mot": rospy.Publisher("enable/motor_control", Bool, queue_size=1),
            "enable_pub_nav": rospy.Publisher("enable/navigation", Bool, queue_size=1),
            "enable_pub_loc": rospy.Publisher("enable/localization", Bool, queue_size=1),
            "enable_pub_obs": rospy.Publisher("enable/obstacles", Bool, queue_size=1)
        }
        self.state_vars = {}

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

        return

    # Configure the enable state for each group
    # Each input parameter should be a boolean
    def set_enables(self, motors=False, nav=False, localization=False, obstacles=False):

        # Create ROS bool messages
        mot_msg = Bool()
        nav_msg = Bool()
        loc_msg = Bool()
        obs_msg = Bool()

        # Set data accordingly
        mot_msg.data = motors
        nav_msg.data = nav
        loc_msg.data = localization
        obs_msg.data = obstacles

        # Publish messages
        self.publishers["enable_pub_mot"].publish(mot_msg)
        self.publishers["enable_pub_nav"].publish(nav_msg)
        self.publishers["enable_pub_loc"].publish(loc_msg)
        self.publishers["enable_pub_obs"].publish(obs_msg)


    ## TODO DEFINE BEHAVIORS FOR EACH STATE ##

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

            # Process any commands we may have received
            self.process_state_changes()

            # Sleep at loop rate
            loop_rate.sleep()

        self.advance_state()

        return

    def state_localizing(self):
        # Enable camera servo node aruco_camera_centering
        return

    def state_drive_to_P0(self):
        return

    def state_drive_to_P1(self):
        return

    def state_drive_to_P2(self):
        return

    def state_drive_to_P3(self):
        return

    def state_teleop_control(self):
        return

    def state_mine_gravel(self):
        return

    def state_approach_bin(self):
        return

    def state_deposit_gravel(self):
        return