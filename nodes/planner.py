#!/usr/bin/python3

import rospy
import sensor_msgs
import std_msgs
from std_msgs.msg import Float64, Float32

from src.cprocessor import CameraProcessor

class PlannerNode:

    def __init__(
            self,
    ) -> None:
        
        self.debug = rospy.get_param("project/PlannerNode/debug", True)
        rospy.loginfo(self.debug)
        self.camera = CameraProcessor(self.debug)

        # Receive camera images
        self.camera_sub = rospy.Subscriber(
            "car/image_raw", sensor_msgs.msg.Image, self._camera_callback
        )

        self.error_pub = rospy.Publisher("error", Float32, queue_size=10)

        rospy.loginfo("Planner initialized")

    def _camera_callback(
            self,
            img_msg
    ) -> None:
        
        centerline = self.camera.process(img_msg)
        rospy.loginfo(centerline)

        #invia l'errore al control node (da calcolare)
        err_msg = Float32()
        err_msg.data = 5.0
        self.error_pub.publish(err_msg)
        rospy.loginfo("error published")
        

if __name__=='__main__':
    rospy.init_node("PlannerNode")
    PlannerNode()
    rospy.spin()
    rospy.loginfo("Planner node shutting down")