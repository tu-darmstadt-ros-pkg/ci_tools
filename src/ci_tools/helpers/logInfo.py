import rospy


class LogInfo(object):
    """Helper methods regarding logging."""

    @staticmethod
    def log(message):
        """Uses rospy.loginfo() to post message with additional"""
        rospy.loginfo("[argo_ci_tools]: " + message)
