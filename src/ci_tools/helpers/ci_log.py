import rospy


class CiLog(object):
    """Helper methods regarding logging."""
    message_template = '[%s]: %s'

    @staticmethod
    def info(message):
        """Uses rospy.loginfo() to post message with node name as prefix"""
        rospy.loginfo(CiLog.message_template % (rospy.get_name(), message))

    @staticmethod
    def error(message):
        """Uses rospy.logerr() to post message with node name as prefix"""
        rospy.logerr(CiLog.message_template % (rospy.get_name(), message))
