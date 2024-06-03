#!/usr/bin/python3

import time
import rospy
import std_msgs

import math

from project.msg._Error_msg import Error_msg

MAX_VELOCITY = 10
ADD_VELOCITY = 0.2

class ControlNode:

    def __init__(
            self,
    ) -> None:
        
        self.P_value = rospy.get_param("control/P_value", 1)
        self.I_value = rospy.get_param("control/I_value", 1)
        self.D_value = rospy.get_param("control/D_value", 1)
        self.time = 0
        self.integral = 0.0
        self.velocity = 0
        self.error = 0
        
        rospy.loginfo("PID params: " )

        self.r_wheel = rospy.Publisher('/car/front_right_velocity_controller/command', std_msgs.msg.Float64, queue_size=10)
        self.l_wheel = rospy.Publisher('/car/front_left_velocity_controller/command', std_msgs.msg.Float64, queue_size=10)
        rospy.loginfo("Control nodes initialized")

        self.sub = rospy.Subscriber("/planner/error", Error_msg, self._pid_callback)

        rospy.loginfo("Error subscribed")
    
    def _pid_callback(self, error):
        x, theta = self._update_error(error.errx, error.errtheta)
        rospy.loginfo(f'error updated')
        l_velocity, r_velocity = self._compute_velocity(x, theta)
        rospy.loginfo(f'velocities computed')
        msg = std_msgs.msg.Float64()
        msg.data = l_velocity
        self.l_wheel.publish(msg)
        msg.data = r_velocity
        self.r_wheel.publish(msg)
        rospy.loginfo(f'velocities published')


    def _update_error(self, errx, errtheta):
        rospy.loginfo(f'x: {errx}, theta: {errtheta}')
        x = self._compute_pid(errx)
        theta = self._compute_pid(errtheta)
 
        rospy.loginfo(f'x_pid: {x}, theta_pid: {theta}')

        return x, theta
    
    def _compute_pid(self, err):
        # Compute time
        current_time = time.time()
        dt = current_time - self.time
        self.time = current_time

        # Compute integral
        self.integral += dt * (err + self.error) / 2

        # Compute PID output
        P = self.P_value * err
        I = self.I_value * self.integral
        D = self.D_value * (err - self.error) / dt
        control = P + I + D

        return control

    def _compute_velocity(
            self,
            x,
            theta
    ):
        o = self.velocity/x

        if self.velocity < MAX_VELOCITY:
            self.velocity += ADD_VELOCITY

        rospy.loginfo(f'v: {self.velocity}, omega: {o}')

        right_velocity = self.velocity - o
        left_velocity = self.velocity + o

        right_velocity = 5
        left_velocity = 6

        rospy.loginfo(f'r_velocity:{right_velocity}, l_velocity: {left_velocity}')

        return right_velocity, left_velocity

    def stop(
            self
    ) -> None:
        

        pass

if __name__=='__main__':
    rospy.init_node("Control", anonymous=True)
    node = ControlNode()
    rospy.loginfo("Control nodes")
    rospy.on_shutdown(node.stop)
    rospy.spin()
    rospy.loginfo("Control node shutting down")