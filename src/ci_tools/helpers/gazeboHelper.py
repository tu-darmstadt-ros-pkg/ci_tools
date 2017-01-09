from std_srvs.srv import Empty
import rospy
from ci_log import CiLog


class GazeboHelper(object):
    """Helper methods regarding Gazebo."""

    @staticmethod
    def unpause_physics():
        """Sends signal to Gazebo to unpause physics."""
        try:
            rospy.wait_for_service('/gazebo/unpause_physics', timeout=60)
        except rospy.ROSException as timeout_exception:
            CiLog.error('Timeout waiting for unpause_physics service:\n%s' % timeout_exception)
            return False

        unpause_physics_client = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)

        try:
            unpause_physics_client()
        except rospy.ServiceException as service_exception:
            CiLog.error('Error calling unpause_physics service:\n%s' % service_exception)
            return False

        CiLog.info("[helpers.GazeboHelper] Gazebo physics unpaused.")
        return True
