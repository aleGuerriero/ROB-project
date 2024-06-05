#!/usr/bin/python3
from __future__ import annotations

import rospy
import sensor_msgs.msg
import std_msgs

import numpy as np
from project.msg._Error_msg import Error_msg
from src.plotter import Plotter
from scripts.errors import ErrorType, ErrorTypeException
import std_msgs.msg
import sensor_msgs

MAX_VELOCITY = 10
ADD_VELOCITY = 0.5

TURNING = 3

_Errors = {
    'position': ErrorType.POSITION,
    'orientation': ErrorType.ORIENTATION,
    'nonlinear': ErrorType.NON_LINEAR
}

class ControlNode:

    def __init__(
            self,
    ) -> None:
        error_type = rospy.get_param("/project/ControlNode/err", "nonlinear")
        if error_type not in _Errors.keys():
            raise ErrorTypeException
        else:
            self.error_type = _Errors[error_type]
        
        self.P_value = rospy.get_param("/project/ControlNode/kp", 1)
        self.I_value = rospy.get_param("/project/ControlNode/ki", 0)
        self.D_value = rospy.get_param("/project/ControlNode/kd", 0)
        
        self.velocity = 0
        
        self.prev_errx = 0
        self.prev_errtheta = 0
        self.time = rospy.get_rostime()
        self.x_integral = 0.0
        self.theta_integral = 0.0

        self.act_vel = [0.0,0.0]
        
        rospy.loginfo(f'Error type: {self.error_type}')
        rospy.loginfo(f"PID params: {self.P_value}, {self.I_value}, {self.D_value}" )

        self.r_wheel = rospy.Publisher('/car/front_right_velocity_controller/command', std_msgs.msg.Float64, queue_size=10)
        self.l_wheel = rospy.Publisher('/car/front_left_velocity_controller/command', std_msgs.msg.Float64, queue_size=10)
        rospy.loginfo("Control nodes initialized")

        self.isgoing = False
        self.started = False
        #self.starter = rospy.Subscriber("starter", std_msgs.msg.Bool, self.start)
            
        self.sub = rospy.Subscriber("planner/error", Error_msg, self._pid_callback)

        rospy.loginfo("Error subscribed")

        self.plot = Plotter()

    def start(self, msg):
        if (msg.data == True):
            self.isgoing = True
            
        return
    
    def _pid_callback(self, error):
        
        #while not self.isgoing:
        #    rospy.loginfo('waiting')
        
        #if not self.started:
        #   self.time= rospy.get_rostime()
        #    self.started = True
        rospy.loginfo(f'error updated')
        x, theta = self._update_error(error.errx, error.errtheta)
        rospy.loginfo(f'error updated')
        l_velocity, r_velocity = self._compute_velocity(x, theta)
        rospy.loginfo(f'velocities computed')

        ac = rospy.Subscriber("/car/joint_states", sensor_msgs.msg.JointState, self.compute_act_velocity)
        self.plot.plot_velocities(r_velocity, l_velocity, MAX_VELOCITY, self.act_vel)

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

    def _update_error(self, errx, errtheta):
        rospy.loginfo(f'x: {errx}, theta: {errtheta}')

        # Compute time
        current_time = rospy.get_rostime()
        dt = (current_time - self.time).to_sec()
        self.time = current_time

        x, self.x_integral = self._compute_pid(dt, errx, self.prev_errx, self.x_integral)
        self.prev_errx = errx
        theta, self.theta_integral = self._compute_pid(dt, errtheta, self.prev_errtheta, self.theta_integral)
        self.prev_errtheta = errtheta
        rospy.loginfo(f'x_integral: {self.x_integral}, theta_integral: {self.theta_integral}')
        rospy.loginfo(f'x_pid: {x}, theta_pid: {theta}')

        return x, theta
    
    def _compute_pid(self, dt, err, prev_err, integral):
        # Compute integral
        if integral < 1.2:
            integral += dt * (err + prev_err) / 2

        # Compute PID output
        P = self.P_value * err
        I = self.I_value * integral
        D = self.D_value * (err - prev_err) / dt
        control = P + I + D

        return control, integral

    def _compute_velocity(
            self,
            x,
            theta
    ):
        v = self.velocity

        if self.velocity < MAX_VELOCITY:
            self.velocity += ADD_VELOCITY
    
        o = self._compute_omega(x, theta)

        rospy.loginfo(f'v: {v}, omega: {o}')

        right_velocity = v + o
        left_velocity = v - o

        #car/joint_states
        #ac
        

        rospy.loginfo(f'r_velocity:{right_velocity}, l_velocity: {left_velocity}')

        return right_velocity, left_velocity
    
    def _compute_omega(
            self,
            x: float | None = None,
            theta: float | None = None
    ) -> float:
        if self.error_type == ErrorType.POSITION:
            return TURNING*x
        if self.error_type == ErrorType.ORIENTATION:
            return TURNING*theta
        if self.error_type == ErrorType.NON_LINEAR:
            return TURNING*theta - x*self.velocity*np.sinc(theta)

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

    
    