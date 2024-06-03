#!/usr/bin/python3

import rospy
import sensor_msgs
import std_msgs

from src.cprocessor import CameraProcessor
from src.strategies import TrajectoryTracking

class PlannerNode:

    def __init__(
            self,
    ) -> None:
        
        self.debug = rospy.get_param("project/PlannerNode/debug", True)

        self.strategy_param = rospy.get_param("project/PlannerNode/strategy", "trajectory")
        if self.strategy_param=="trajectory":
            self.strategy = TrajectoryTracking()

        rospy.loginfo(self.debug)
        self.camera = CameraProcessor(self.debug)

        # Receive camera images
        self.camera_sub = rospy.Subscriber(
            "car/image_raw", sensor_msgs.msg.Image, self._camera_callback
        )

        self.error_pub = rospy.Publisher("/planner/error", std_msgs.msg.Float32, queue_size=1)

        rospy.loginfo("Planner initialized")

    def _camera_callback(
            self,
            img_msg
    ) -> None:
        
        pos, centerline = self.camera.process(img_msg)
        rospy.loginfo(centerline)

        #invia l'errore al control node (da calcolare)
        err_msg = std_msgs.msg.Float32()
        errx, errtheta = self.strategy.plan(pos, centerline)
        err_msg.data = errx/320 - 1
        self.error_pub.publish(err_msg)
        rospy.loginfo("error published")
        

if __name__=='__main__':
    rospy.init_node("PlannerNode")
    PlannerNode()
    rospy.spin()
    rospy.loginfo("Planner node shutting down")