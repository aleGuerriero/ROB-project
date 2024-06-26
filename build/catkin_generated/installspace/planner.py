#!/usr/bin/python3
from __future__ import annotations
import rospy
import sensor_msgs.msg
from project.msg._Error_msg import Error_msg
from src.cprocessor import CameraProcessor
from src.strategies.trajectory import TrajectoryTracking

class PlannerNode:

    def __init__(
            self,
    ) -> None:
        
        self.debug = rospy.get_param("/project/PlannerNode/debug", False)
        self.camera = CameraProcessor(self.debug)
        
        self.strategy_param = rospy.get_param("/project/PlannerNode/strategy", "trajectory")
        if self.strategy_param=="trajectory":
            self.strategy = TrajectoryTracking()
                        
        # Receive camera images
        self.camera_sub = rospy.Subscriber("/car/image_raw", sensor_msgs.msg.Image, self._camera_callback)
        rospy.loginfo("Error subscribed")
        rospy.loginfo("PID control node initialized")

        self.error_pub = rospy.Publisher("/planner/error", Error_msg, queue_size=1)


    def _camera_callback(self, img_msg):

        #process the image
        pos, crosshair, centerline = self.camera.process(img_msg)
        #find errx and errhteta 
        waypoint, errx, theta, errtheta = self.strategy.plan(crosshair, self.camera.width, centerline)

        if True:
            self.camera.draw(pos, crosshair, waypoint)
            self.camera.show()

        self.send_error(errx, errtheta)
        return
        
       
    def send_error(self, errx, errtheta):       

        #send errors to ControllerPID
        msg = Error_msg()
        msg.errx = errx
        msg.errtheta = errtheta
        rospy.loginfo(f'Publishing x: {msg.errx}, theta: {msg.errtheta}')
        self.error_pub.publish(msg)
        self.error_pub.publish(msg)
        return      
        

if __name__=='__main__':
    rospy.init_node("PlannerNode")
    node = PlannerNode()
    rospy.loginfo("Planner node")
    rospy.loginfo("Planner node shutting down.")
    rospy.spin()