#!/usr/bin/python3

import rospy
import sensor_msgs
import std_msgs
from std_msgs.msg import Float64, Float32

from src.cprocessor import CameraProcessor
<<<<<<< HEAD
=======
from src.strategies import TrajectoryTracking
from project.msg._Error_msg import Error_msg
>>>>>>> 05f48a861eebffabdfa07944d6cd93771ac76466

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

<<<<<<< HEAD
        self.error_pub = rospy.Publisher("error", Float32, queue_size=10)
=======
        self.error_pub = rospy.Publisher("/planner/error", Error_msg, queue_size=1)
>>>>>>> 05f48a861eebffabdfa07944d6cd93771ac76466

        rospy.loginfo("Planner initialized")

    def _camera_callback(
            self,
            img_msg
    ) -> None:
        
        centerline = self.camera.process(img_msg)
        rospy.loginfo(centerline)

        #invia l'errore al control node (da calcolare)
<<<<<<< HEAD
        err_msg = Float32()
        err_msg.data = 5.0
=======
        err_msg = Error_msg()
        errx, errtheta = self.strategy.plan(pos, centerline)
        rospy.loginfo(f'Computed x: {errx}, theta: {errtheta}')
        err_msg.errx = (errx + 639)/639 - 1
        err_msg.errtheta = errtheta
        rospy.loginfo(f'Publishing x: {err_msg.errx}, theta: {err_msg.errtheta}')
>>>>>>> 05f48a861eebffabdfa07944d6cd93771ac76466
        self.error_pub.publish(err_msg)
        rospy.loginfo("error published")
        

if __name__=='__main__':
    rospy.init_node("PlannerNode")
    PlannerNode()
    rospy.spin()
    rospy.loginfo("Planner node shutting down")