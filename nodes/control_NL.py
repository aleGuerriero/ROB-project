#!/usr/bin/python3
from __future__ import annotations
import rospy
import std_msgs.msg
import numpy as np
from project.msg._Error_msg import Error_msg

MAX_VELOCITY = 3.3
ADD_VELOCITY = 0.1

WHEELR = 0.25
WHEELD = 1.4

class ControlNLNode:

    def __init__(
            self,
    ) -> None:
                        
        self.velocity = 0      
        self.r_wheel = rospy.Publisher('/car/front_right_velocity_controller/command', std_msgs.msg.Float64, queue_size=10)
        self.l_wheel = rospy.Publisher('/car/front_left_velocity_controller/command', std_msgs.msg.Float64, queue_size=10)

        # Receive error message rom control_PID node
        self.sub = rospy.Subscriber("control_PID/error", Error_msg, self.send_error, queue_size=1)
        rospy.loginfo("Error subscribed")
        
    def send_error(self, msg:Error_msg):        

        errx, errtheta = msg.errx, msg.errtheta 
        rospy.loginfo(f'Error: {errx}, {errtheta}')

        l_velocity, r_velocity = self._compute_velocity(errx, errtheta)
        rospy.loginfo(f'L_velocity: {l_velocity}, R_velocity: {errtheta}')

        msg = std_msgs.msg.Float64()
        msg.data = l_velocity
        self.l_wheel.publish(msg)
        msg.data = r_velocity
        self.r_wheel.publish(msg)
        rospy.loginfo(f'velocities published')
        return
    
    def _compute_velocity(self, errx, errtheta):

        if self.velocity < MAX_VELOCITY:
            self.velocity += ADD_VELOCITY

        eq = (errx - 0.2*errtheta) - errx*self.velocity*np.sinc(errtheta)
        right_velocity = (2*self.velocity - WHEELD*eq)/(2*WHEELR)
        left_velocity = (2*self.velocity + WHEELD*eq)/(2*WHEELR)
        rospy.loginfo(f'r_velocity:{right_velocity}, l_velocity: {left_velocity}')
        return right_velocity, left_velocity
        
    def stop(self):
        msg = std_msgs.msg.Float64()
        msg.data = 0
        # Send the messages multiple times since they are not
        #   guaranteed to be delivered. Ugly, but it seems to work.
        for _ in range(10):
            self.l_wheel.publish(msg)
            self.r_wheel.publish(msg)
        
        rospy.loginfo("Control node shutting down.")
        

if __name__=='__main__':
    rospy.init_node("ControlNLNode")
    node = ControlNLNode()
    rospy.loginfo("Control nodes")
    rospy.on_shutdown(node.stop)
    rospy.spin()