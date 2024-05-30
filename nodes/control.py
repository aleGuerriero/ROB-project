#!/usr/bin/python3

import time
from std_msgs.msg import String
import rospy
import std_msgs

MAX_VELOCITY = 5
ADD_VELOCITY = 0.5
TURNING = 5 

'''import rospy
from std_msgs.msg import Float64

def publish_velocity():
    # Inizializza il nodo ROS
    rospy.init_node('velocity_publisher', anonymous=True)
    
    # Crea un publisher per il topic specificato
    pub = rospy.Publisher('/car/front_right_velocity_controller/command', Float64, queue_size=10)
    
    # Imposta la frequenza di pubblicazione a 10 Hz
    rate = rospy.Rate(10) # 10 Hz
    
    # Crea un messaggio Float64
    velocity_command = Float64()
    velocity_command.data = 3.0
    
    # Pubblica il messaggio finché ROS è attivo
    while not rospy.is_shutdown():
        # Log del messaggio per debug
        rospy.loginfo(f"Publishing velocity command: {velocity_command.data}")
        
        # Pubblica il messaggio
        pub.publish(velocity_command)
        
        # Dorme per mantenere il loop alla frequenza desiderata
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_velocity()
    except rospy.ROSInterruptException:
        pass'''

class ControlNode:

    def __init__(
            self,
    ) -> None:
        
        self.P_value = rospy.get_param("control/P_value", 1)
        self.I_value = rospy.get_param("control/I_value", 1)
        self.D_value = rospy.get_param("control/D_value", 1)

        self.time = 0
        self.setpoint = 0
        self.thrust = 0
        self.error = 0
        
        rospy.loginfo("PID params: " )

        self.r_wheel = rospy.Publisher('/car/front_right_velocity_controller/command', std_msgs.msg.Float64, queue_size=10)
        self.l_wheel = rospy.Publisher('/car/front_left_velocity_controller/command', std_msgs.msg.Float64, queue_size=10)
        rospy.loginfo("Control nodes initialized")

        self.sub = rospy.Subscriber("/planner/error", std_msgs.msg.Float32, self._pid_callback)

        rospy.loginfo("Error subscribed")
    
    def _pid_callback(self, error):
        msg = std_msgs.msg.Float64()

        l_velocity, r_velocity = self._compute_velocity(
            self._update_error(error.data)
        )

        msg.data = l_velocity
        self.r_wheel.publish(msg)
        msg.data = r_velocity
        self.r_wheel.publish(msg)


    def _update_error(self, err):

        # Compute time
        current_time = time.time()
        dt = current_time - self.time
        self.time = current_time

        # Compute PID output
        P = self.P_value * err
        #I = self.I_value * self.accumulated_integral
        D = self.D_value * (err - self.error) / dt
        #control = P + I + D
        control = P + D

        return control

    def _compute_velocity(
            self,
            control
    ):
        if self.thrust < MAX_VELOCITY:
            self.thrust += ADD_VELOCITY

        right_velocity = self.thrust - TURNING * control
        left_velocity = self.thrust + TURNING * control

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