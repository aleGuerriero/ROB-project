#!/usr/bin/python3

import random
import rospy
import sensor_msgs
import std_msgs

from src.cprocessor import CameraProcessor
from src.strategies import TrajectoryTracking
from project.msg._Error_msg import Error_msg
import std_msgs.msg
from src.plotter import Plotter

class PlannerNode:

    def __init__(
            self,
    ) -> None:
        
        self.debug = rospy.get_param("project/PlannerNode/debug", True)

        self.strategy_param = rospy.get_param("project/PlannerNode/strategy", "trajectory")
        if self.strategy_param=="trajectory":
            self.strategy = TrajectoryTracking()

        self.camera = CameraProcessor(self.debug)

        # Receive camera images
        self.camera_sub = rospy.Subscriber(
            "car/image_raw", sensor_msgs.msg.Image, self._camera_callback
        )

        self.error_pub = rospy.Publisher("/planner/error", Error_msg, queue_size=1)

        self.plot = Plotter()

        

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
        
        #Plot       
        self.plot.plot_errors(errx, errtheta)
    
        if True:
            self.camera.draw(pos, crosshair, waypoint)
            self.camera.show()

    
        

if __name__=='__main__':
    rospy.init_node("PlannerNode")
    PlannerNode()
    rospy.spin()
    rospy.loginfo("Planner node shutting down")