from std_srvs.srv import Empty
import rospy
from logInfo import LogInfo


class GazeboHelper(object):
    """Helper methods regarding Gazebo."""

    @staticmethod
    def unpause_physics():
        """Sends signal to Gazebo to unpause physics."""
        physics_client = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        physics_client()
        LogInfo.log("[helpers.GazeboHelper] Gazebo physics unpaused.")
