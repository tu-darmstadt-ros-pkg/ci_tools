import threading
import time
import rospy
from flexbe_msgs.msg import BEStatus

from helpers.flexBEHelper import FlexBEHelper
from helpers.gazeboHelper import GazeboHelper
from helpers.ci_log import CiLog


class SimulationControl(object):
    """Class for controlling the simulation of a specified scenario."""

    def __init__(self):
        """Initialization of variables and reading ros parameters."""
        self._max_sim_time_reached = False
        self._max_wall_time_reached = False
        self._behavior_finished = False
        self._flexbe_status_subscriber = None

        self._mission_finalizers = ""
        self._mission_sim_time_in_sec = 0
        self._finalizer_functions = []

        self.read_ros_params()
        CiLog.info("Init of SimulationControl constructor finished.")

    def read_ros_params(self):
        """Reads parameters that have been set in roslaunch file."""
        mission_finalizers_param_name = rospy.search_param('mission_finalizers')
        self._mission_finalizers = str(rospy.get_param(mission_finalizers_param_name))

        mission_time_full_param_name = rospy.search_param('mission_sim_time')
        self._mission_sim_time_in_sec = int(rospy.get_param(mission_time_full_param_name))

    def import_finalizers(self,):
        """Import of finalizer classes that will be executed."""
        if len(self._mission_finalizers) > 0:
            mission_finalizers_list = self._mission_finalizers.split(",")
            for mission_finalizer in mission_finalizers_list:
                module_name, finalizer = mission_finalizer.rsplit(".", 1)
                try:
                    module = __import__('finalizers.' + module_name, fromlist=[finalizer])
                    self._finalizer_functions.append(getattr(module, finalizer))
                except Exception as e:
                    CiLog.error('Could not import finalizer "%s":\n%s' % (mission_finalizer, e))

    def start_sim(self):
        """Unpauses gazebo."""
        time.sleep(20)
        if not GazeboHelper.unpause_physics():
            return False
        time.sleep(30)
        return True

    def start_behavior(self, behavior_to_start):
        """Tries to start FlexBE behavior."""
        time.sleep(20)
        if not FlexBEHelper.start_flexbe_behavior(behavior_to_start):
            return False
        time.sleep(5)
        return True

    def start_simulation_timer(self):
        """Starts callback, that sets _max_sim_time_reached if time over."""
        rospy.Timer(rospy.Duration(self._mission_sim_time_in_sec), self.callback_max_sim_time_reached, oneshot=True)
        # create a wall clock timer that has a 4 times longer timeout (= gazebo real time factor 0.25)
        # this is needed as backup if gazebo fails to start the simulation time
        threading.Timer(4 * self._mission_sim_time_in_sec, self.callback_max_wall_time_reached)
        CiLog.info("Simulation timer started. Simtime is %s" % self._mission_sim_time_in_sec)

    def monitor_behavior_status(self):
        """Subscribe flexbe status topic to be notified when behavior has stopped."""
        self._flexbe_status_subscriber = rospy.Subscriber('/flexbe/status', BEStatus, self.callback_flexbe_status)

    def perform_final_actions(self):
        """Calls all imported finalizer_functions."""
        for finalizer_function in self._finalizer_functions:
            finalizer_function()

    def is_simulation_finished(self):
        """Returns true if maximum simulation or wall time have been reached or the behavior has finished."""
        return self._max_sim_time_reached or self._max_wall_time_reached or self._behavior_finished

    def callback_flexbe_status(self, msg):
        """When flexbe status is FINISHED or FAILED, set _behavior_finished flag."""
        self._behavior_finished = msg.code in (BEStatus.FINISHED, BEStatus.FAILED)

    def callback_max_sim_time_reached(self, event):
        """Callback that is used to indicate when maximum simulation_time is reached."""
        self.perform_final_actions()
        self._max_sim_time_reached = True
        CiLog.info("Simulation timer finished.")

    def callback_max_wall_time_reached(self, event):
        """Callback that is used to indicate when maximum wall time is reached."""
        self.perform_final_actions()
        self._max_wall_time_reached = True
