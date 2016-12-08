from std_srvs.srv import Empty
import rospy
from ciLogInfo import CiLogInfo


class GazeboHelper(object):
    """Helper methods regarding Gazebo."""

    @staticmethod
    def unpause_physics():
        """Sends signal to Gazebo to unpause physics."""
        rospy.wait_for_service('/gazebo/unpause_physics')
        physics_client = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        physics_client()
        CiLogInfo.log("[helpers.GazeboHelper] Gazebo physics unpaused.")
