import rospy


class CiLogInfo(object):
    """Helper methods regarding logging."""

    @staticmethod
    def log(message):
        """Uses rospy.loginfo() to post message with additional"""
        rospy.loginfo("[ci_tools]: " + message)
