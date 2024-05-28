#!/usr/bin/python3

import rospy
import sensor_msgs
import std_msgs

from src.cprocessor import CameraProcessor

class PlannerNode:

    def __init__(
            self,
    ) -> None:
        
        self.debug = rospy.get_param("project/PlannerNode/debug", False)
        self.camera = CameraProcessor(self.debug)

        # Receive camera images
        self.camera_sub = rospy.Subscriber(
            "car/image_raw", sensor_msgs.msg.Image, self._camera_callback
        )

        rospy.loginfo("Planner initialized")

    def _camera_callback(
            self,
            img_msg
    ) -> None:
        
        self.camera.process(img_msg)

if __name__=='__main__':
    rospy.init_node("Planner")
    PlannerNode()
    rospy.spin()
    rospy.loginfo("Planner node shutting down")