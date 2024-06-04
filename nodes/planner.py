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
        
        self.debug = rospy.get_param("/project/PlannerNode/debug", False)

        self.strategy_param = rospy.get_param("/project/PlannerNode/strategy", "trajectory")
        if self.strategy_param=="trajectory":
            self.strategy = TrajectoryTracking()

        self.camera = CameraProcessor(self.debug)

        # Receive camera images
        self.camera_sub = rospy.Subscriber(
            "/car/image_raw", sensor_msgs.msg.Image, self._camera_callback
        )

        self.error_pub = rospy.Publisher("planner/error", Error_msg, queue_size=1)

        rospy.loginfo("Planner initialized")

    def _camera_callback(
            self,
            img_msg
    ) -> None:
        
        pos, crosshair, centerline = self.camera.process(img_msg)

        #invia l'errore al control node (da calcolare)
        err_msg = Error_msg()
        waypoint, errx, errtheta = self.strategy.plan(crosshair, self.camera.width, centerline)
        err_msg.errx = errx
        err_msg.errtheta = errtheta
        rospy.loginfo(f'Publishing x: {err_msg.errx}, theta: {err_msg.errtheta}')
        self.error_pub.publish(err_msg)

        if self.debug:
            self.camera.draw(pos, crosshair, waypoint)
            self.camera.show()
        

if __name__=='__main__':
    rospy.init_node("PlannerNode")
    PlannerNode()
    rospy.spin()
    rospy.loginfo("Planner node shutting down")