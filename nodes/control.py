#!/usr/bin/python3

import rospy
import std_msgs

class ControlNode:

    def __init__(
            self
    ) -> None:
        pass

    def stop(
            self
    ) -> None:
        pass

if __name__=='__main__':
    rospy.init("Control")
    node = ControlNode()
    rospy.on_shutdown(node.stop)
    rospy.spin()
    rospy.loginfo("Control node shutting down")