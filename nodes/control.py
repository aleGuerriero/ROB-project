#!/usr/bin/python3
from __future__ import annotations

import rospy
import std_msgs
import numpy as np
from project.msg._Error_msg import Error_msg
from src.plotter import Plotter
from scripts.errors import ErrorType, ErrorTypeException
import std_msgs.msg

MAX_VELOCITY = 3.4
ADD_VELOCITY = 0.1

WHEELR = 0.25
WHEELD = 1.4

_Errors = {
    'position': ErrorType.POSITION,
    'orientation': ErrorType.ORIENTATION,
    'linear': ErrorType.LINEAR,
    'nonlinear': ErrorType.NON_LINEAR
}

class ControlNode:

    def __init__(
            self,
    ) -> None:
        error_type = rospy.get_param("/project/ControlNode/err", "position")
        if error_type not in _Errors.keys():
            raise ErrorTypeException
        else:
            self.error_type = _Errors[error_type]
        
        self.P_value = rospy.get_param("/project/ControlNode/kp", 1)
        self.I_value = rospy.get_param("/project/ControlNode/ki", 0)
        self.D_value = rospy.get_param("/project/ControlNode/kd", 0)
        
        self.velocity = 0
        
        self.prev_dx = 0
        self.prev_dtheta = 0
        self.time = 0
        self.dx_integral = 0.0
        self.dtheta_integral = 0.0

        self.x_derivative = 0
        self.theta_derivative = 0

        self.running = False

        self.act_vel = [0.0,0.0]
        
        rospy.loginfo(f'Error type: {self.error_type}')
        rospy.loginfo(f"PID params: {self.P_value}, {self.I_value}, {self.D_value}" )

        self.r_wheel = rospy.Publisher('/car/front_right_velocity_controller/command', std_msgs.msg.Float64, queue_size=10)
        self.l_wheel = rospy.Publisher('/car/front_left_velocity_controller/command', std_msgs.msg.Float64, queue_size=10)
        rospy.loginfo("Control nodes initialized")

        self.isgoing = False
        self.started = False

        self.plot = Plotter()
        
        self.sub = rospy.Subscriber("planner/error", Error_msg, self._pid_callback, queue_size=1)

        rospy.loginfo("Error subscribed")

    def start(self, msg):
        if (msg.data == True):
            self.isgoing = True
        return
    
    def _pid_callback(
            self,
            error: Error_msg
    ) -> None:        

        errx, errtheta = self._update_error(error)
        l_velocity, r_velocity = self._compute_velocity(errx, errtheta)


        #ac = rospy.Subscriber("/car/joint_states", sensor_msgs.msg.JointState, self.compute_act_velocity)
        

        msg = std_msgs.msg.Float64()
        msg.data = l_velocity
        self.l_wheel.publish(msg)
        msg.data = r_velocity
        self.r_wheel.publish(msg)
        rospy.loginfo(f'velocities published')
    
    def compute_act_velocity(self, a):
        self.act_vel[0] = a.velocity[0]
        self.act_vel[1] = a.velocity[1]
        rospy.loginfo(f'l: {self.act_vel[0]} r: {self.act_vel[1]}')
        return

    def _update_error(
            self, 
            error: Error_msg
    ) -> float:
        errx, errtheta = error.errx, error.errtheta
        rospy.loginfo(f'Error: {errx}, {errtheta}')

        if not self.running:
            self.time = rospy.get_rostime()
            self.running = True

        # Compute time
        current_time = rospy.get_rostime()
        dt = (current_time - self.time).to_sec()
        if dt!=0:
            # Compute integral
            self.dx_integral = ControlNode._integrate(self.dx_integral, dt, errx, self.prev_dx)
            self.dtheta_integral = self._integrate(self.dtheta_integral, dt, errtheta, self.prev_dtheta)
            #Compute derivative
            self.x_derivative = (errx - self.prev_dx) / dt
            self.theta_derivative = (errtheta - self.prev_dtheta) / dt

        self.time = current_time

        # Compute PID output
        Px, Pt = (self.P_value * errx, self.P_value * errtheta)
        Ix, It = (self.I_value * self.dx_integral, self.I_value * self.dtheta_integral)
        Dx, Dt = (self.D_value * self.x_derivative, self.D_value * self.theta_derivative)
        controlx = Px + Ix + Dx
        controltheta = Pt + It + Dt

        rospy.loginfo(f'PID: {controlx}, {controltheta}')

        return controlx, controltheta
    
    @staticmethod
    def _integrate(
            integral,
            dt,
            eq,
            prev_eq
    ) -> float:
        if integral < 1.2:
            integral += dt * (eq + prev_eq) / 2

        rospy.loginfo(f'Integral: {integral}')
        return integral

    def _compute_velocity(
            self,
            errx,
            errtheta
    ):
        if self.velocity < MAX_VELOCITY:
            self.velocity += ADD_VELOCITY

        if self.error_type is ErrorType.POSITION:
            eq = errx
        if self.error_type is ErrorType.ORIENTATION:
            eq = errtheta
        if self.error_type is ErrorType.LINEAR:
            eq = errx + errtheta
        if self.error_type is ErrorType.NON_LINEAR:
            eq = (errx - 0.2*errtheta) - errx*self.velocity*np.sinc(errtheta)


        right_velocity = (2*self.velocity - WHEELD*eq)/(2*WHEELR)
        left_velocity = (2*self.velocity + WHEELD*eq)/(2*WHEELR)
        
        self.plot.plot_velocities(right_velocity, left_velocity)

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
    rospy.init_node("ControlNode")
    node = ControlNode()
    rospy.loginfo("Control nodes")
    rospy.on_shutdown(node.stop)
    rospy.spin()

    
    