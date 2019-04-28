import rospy
from geometry_msgs.msg import Twist

class HighLevelController:
    def __init__(self, starting_state = "idle"):
        # Initialize current state to the starting state
        self.current_state = starting_state

        self.p0 = [0,0]
        self.p1 = [0,0]
        self.p2 = [3,3]
        self.p3 = [0,0]

        self.publishers = {
            "drive_cmd" : rospy.Publisher('drive_cmd', Twist, queue_size=1)
        }
        self.state_vars = {}

    def process_state_changes(self):
        # Check for any control commands, loss of localization, etc.
        return

    def advance_state(self):
        # Considering the curren state and other info (such as elapsed time), advance to the correct next state
        return

    def run_state(self):
        return

    ## TODO DEFINE BEHAVIORS FOR EACH STATE ##
    def state_idle(self):
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