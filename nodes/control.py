#!/usr/bin/python3

import time
from std_msgs.msg import String
import rospy
import std_msgs
from std_msgs.msg import Float64, Float32

MAX_VELOCITY = 6
ADD_VELOCITY = 0.05
TURNING = 6

class ControlNode:

    def __init__(
            self,
    ) -> None:
        
        self.KP_value = rospy.get_param("control/P_value", 1)
        self.KI_value = rospy.get_param("control/I_value", 1)
        self.KD_value = rospy.get_param("control/D_value", 1)
        self.prev_error = 0.0
        self.time = 0
        self.integral = 0.0
        self.thrust = 0
        rospy.loginfo("PID params inizialized")

        self.r_wheel = rospy.Publisher('/car/front_right_velocity_controller/command', Float64, queue_size=10)
        self.l_wheel = rospy.Publisher('/car/front_left_velocity_controller/command', Float64, queue_size=10)
        rospy.loginfo("Control nodes initialized")

        self.sub = rospy.Subscriber("error", Float32, self.error_update)
        rospy.loginfo("Error subscribed")

        return
    

    def error_update(self, err):

        current_time = rospy.get_rostime().to_sec()
        if (current_time==0):
            return
        
        error = err.data

        dt = (current_time - self.time)
        self.time = current_time
        # Approximate the integral using trapezoids
        self.integral += dt * (error + self.prev_error) / 2
        # Compute PID output
        KP = self.KP_value * error
        KI = self.KI_value * self.integral
        KD = self.KD_value * (error - self.prev_error) / dt
        control = KP + KI + KD
        rospy.loginfo(f"Control (PID) calculated: {control}")
    
        if self.thrust < MAX_VELOCITY:
            self.thrust += ADD_VELOCITY

        right_velocity = self.thrust #self.thrust - TURNING * control
        left_velocity = self.thrust + 0.7 #self.thrust + TURNING * control
        self.car_command(right_velocity, left_velocity)
    

    def car_command(self, right_velocity, left_velocity):
        msg = Float64()

        msg.data = right_velocity
        rospy.loginfo(f"Publishing r_velocity command: {msg.data}")
        self.r_wheel.publish(msg)

        msg.data = left_velocity
        rospy.loginfo(f"Publishing l_velocity command: {msg.data}")
        self.l_wheel.publish(msg)

        rospy.loginfo("Commands published")

        return

        """
        Publish the comand with provided velocities.

        Args:
            right_velocity: right wheel speed
            left_velocity: left wheel speed

        Returns:
            None
        """

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