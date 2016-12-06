import time
import rospy
from helpers.flexBEHelper import FlexBEHelper
from helpers.gazeboHelper import GazeboHelper


class SimulationControl(object):
    """Class for controlling the simulation of a specified scenario."""

    def __init__(self):
        """Initialization of variables and reading ros parameters."""
        self._max_sim_time_reached = False
        self._mission_finalizers = ""
        self._mission_sim_time_in_sec = 0
        self._mission_behavior = ""
        self._finalizer_classes = []

        self.read_ros_params()

    def read_ros_params(self):
        """Reads parameters that have been set in roslaunch file."""
        mission_finalizers_param_name = rospy.search_param('mission_finalizers')
        self._mission_finalizers = str(rospy.get_param(mission_finalizers_param_name))

        mission_time_full_param_name = rospy.search_param('mission_sim_time')
        self._mission_sim_time_in_sec = int(rospy.get_param(mission_time_full_param_name))

        mission_behavior_full_param_name = rospy.search_param('mission_behavior')
        self._mission_behavior = str(rospy.get_param(mission_behavior_full_param_name))

    def start_sim_with_behavior(self):
        """Unpauses Gazebo, sends behavior to FlexBE and starts simulation timer."""
        time.sleep(20)
        GazeboHelper.unpause_physics()
        time.sleep(20)
        FlexBEHelper.start_flexbe_behavior(self._mission_behavior)
        rospy.Timer(rospy.Duration(self._mission_sim_time_in_sec), self._stop_node)

    def start_sim(self):
        """Unpauses gazebo."""
        time.sleep(20)
        GazeboHelper.unpause_physics()
        time.sleep(5)

    def start_behavior(self, behavior_to_start):
        """Tries to start FlexBE behavior."""
        time.sleep(20)
        FlexBEHelper.start_flexbe_behavior(behavior_to_start)
        time.sleep(5)

    def start_simulation_timer(self):
        """Starts callback, that sets _max_sim_time_reached if time over."""
        rospy.Timer(rospy.Duration(self._mission_sim_time_in_sec), self._stop_node)

    def perform_final_actions(self):
        """Calls final_action methods of all imported finalizers."""
        for finalizer_class in self._finalizer_classes:
            finalizer_class.final_action()

    def max_sim_time_reached(self):
        """Returns if maximum simulation time has been reached."""
        if self._max_sim_time_reached:
            return True
        else:
            return False

    def _stop_node(self, event):
        """Callback that is used to indicate when maximum simulation_time is readed."""
        self._max_sim_time_reached = True
