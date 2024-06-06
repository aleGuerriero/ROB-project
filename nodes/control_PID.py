#!/usr/bin/python3
from __future__ import annotations
import sensor_msgs
import rospy
from src.cprocessor import CameraProcessor
from src.strategies.trajectory import TrajectoryTracking
import std_msgs
from project.msg._Error_msg import Error_msg
from scripts.errors import ErrorType, ErrorTypeException
import std_msgs.msg

MAX_VELOCITY = 3.6
ADD_VELOCITY = 0.1

WHEELR = 0.25
WHEELD = 1.4

_Errors = {
    'linear': ErrorType.LINEAR,
    'nonlinear': ErrorType.NON_LINEAR
}

class ControlPIDNode:

    def __init__(
            self,
    ) -> None:
        error_type = rospy.get_param("/project/ControlPIDNode/err", "linear")
        if error_type not in _Errors.keys():
            raise ErrorTypeException
        else:
            self.error_type = _Errors[error_type]

        rospy.loginfo(f'Error type: {self.error_type}')
        
        if self.error_type is ErrorType.LINEAR:
            self.P_value = rospy.get_param("/project/ControlPIDNode/kp", 1)
            self.I_value = rospy.get_param("/project/ControlPIDNode/ki", 0)
            self.D_value = rospy.get_param("/project/ControlPIDNode/kd", 0)
        if self.error_type is ErrorType.NON_LINEAR:
            self.P_value = rospy.get_param("/project/ControlPIDNode/kpn", 1)
            self.I_value = rospy.get_param("/project/ControlPIDNode/kin", 0)
            self.D_value = rospy.get_param("/project/ControlPIDNode/kdn", 0)        
        
        rospy.loginfo(f"PID params: {self.P_value}, {self.I_value}, {self.D_value}" )
        
        self.errx = 0
        self.errtheta = 0

        self.velocity = 0
        
        self.prev_dx = 0
        self.prev_dtheta = 0
        self.time = 0
        self.dx_integral = 0.0
        self.dtheta_integral = 0.0

        self.x_derivative = 0
        self.theta_derivative = 0

        self.running = False    

        self.r_wheel = rospy.Publisher('/car/front_right_velocity_controller/command', std_msgs.msg.Float64, queue_size=10)
        self.l_wheel = rospy.Publisher('/car/front_left_velocity_controller/command', std_msgs.msg.Float64, queue_size=10)
        self.error_pub = rospy.Publisher("control_PID/error", Error_msg, queue_size=1)

        
        self.isgoing = False
        self.started = False

        self.strategy_param = rospy.get_param("/project/ControlPIDNode/strategy", "trajectory")
        if self.strategy_param=="trajectory":
            self.strategy = TrajectoryTracking()
        
        self.debug = rospy.get_param("/project/ControlPIDNode/debug", False)
        self.camera = CameraProcessor(self.debug)

        # Receive camera images
        self.camera_sub = rospy.Subscriber("/car/image_raw", sensor_msgs.msg.Image, self._camera_callback)
        rospy.loginfo("Error subscribed")
        rospy.loginfo("PID control node initialized")

    def _camera_callback(self, img_msg):
        
        pos, crosshair, centerline = self.camera.process(img_msg)

        waypoint, errx, theta, errtheta = self.strategy.plan(crosshair, self.camera.width, centerline)
        self.errx = errx
        self.errtheta = errtheta
        
        if True:
            self.camera.draw(pos, crosshair, waypoint)
            self.camera.show()
        
        self.send_error()
        
    def send_error (self):        

        errx, errtheta = self._update_error()
        l_velocity, r_velocity = self._compute_velocity(errx, errtheta)

        if self.error_type is ErrorType.LINEAR:
            #send directly
            msg = std_msgs.msg.Float64()
            msg.data = l_velocity
            self.l_wheel.publish(msg)
            msg.data = r_velocity
            self.r_wheel.publish(msg)
            rospy.loginfo(f'velocities published')
        
        if self.error_type is ErrorType.NON_LINEAR:
            #send to NON_LINEAR controller
            msg = Error_msg()
            msg.errx = errx
            msg.errtheta = errtheta
            rospy.loginfo(f'Publishing x: {msg.errx}, theta: {msg.errtheta}')
            self.error_pub.publish(msg)
        
    
    def _update_error(self):
        
        rospy.loginfo(f'Error: {self.errx}, {self.errtheta}')

        if not self.running:
            self.time = rospy.get_rostime()
            self.running = True

        # Compute time
        current_time = rospy.get_rostime()
        dt = (current_time - self.time).to_sec()
        if dt!=0:
            # Compute integral
            self.dx_integral = ControlPIDNode._integrate(self.dx_integral, dt, self.errx, self.prev_dx)
            self.dtheta_integral = self._integrate(self.dtheta_integral, dt, self.errtheta, self.prev_dtheta)
            #Compute derivative
            self.x_derivative = (self.errx - self.prev_dx) / dt
            self.theta_derivative = (self.errtheta - self.prev_dtheta) / dt

        self.time = current_time

        # Compute PID output
        Px, Pt = (self.P_value * self.errx, self.P_value * self.errtheta)
        Ix, It = (self.I_value * self.dx_integral, self.I_value * self.dtheta_integral)
        Dx, Dt = (self.D_value * self.x_derivative, self.D_value * self.theta_derivative)
        controlx = Px + Ix + Dx
        controltheta = Pt + It + Dt

        rospy.loginfo(f'PID: {controlx}, {controltheta}')

        return controlx, controltheta
    
    def _compute_velocity(
            self,
            errx,
            errtheta
    ):
        if self.velocity < MAX_VELOCITY:
            self.velocity += ADD_VELOCITY

        eq = errx + errtheta
        right_velocity = (2*self.velocity - WHEELD*eq)/(2*WHEELR)
        left_velocity = (2*self.velocity + WHEELD*eq)/(2*WHEELR)

        rospy.loginfo(f'r_velocity:{right_velocity}, l_velocity: {left_velocity}')

        return right_velocity, left_velocity
    
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
    rospy.init_node("ControlPIDNode")
    node = ControlPIDNode()
    rospy.loginfo("Control nodes")
    rospy.on_shutdown(node.stop)
    rospy.spin()