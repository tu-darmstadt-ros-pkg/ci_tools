from flexbe_msgs.msg import BehaviorExecutionActionGoal
import rospy
import time
from ciLogInfo import CiLogInfo


class FlexBEHelper(object):
    """Helper methods regarding FlexBE."""

    @staticmethod
    def start_flexbe_behavior(behavior_name):
        """Publish message to FlexBE to execute new behavior.

        :param behavior_name: Name of the behavior that will be started.
        """
        behavior_publisher = rospy.Publisher('/flexbe/execute_behavior/goal', BehaviorExecutionActionGoal, queue_size=5)
        time.sleep(5)
        msg = BehaviorExecutionActionGoal()
        msg.goal.behavior_name = behavior_name
        behavior_publisher.publish(msg)
        CiLogInfo.log("[helpers.FlexBEHelper]: New behavior " + behavior_name + " sent to FlexBE.")
