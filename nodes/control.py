#!/usr/bin/python3

import time
from std_msgs.msg import String
import rospy
import std_msgs
from std_msgs.msg import Float64, Float32

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
        
        rospy.loginfo("PID params: " )

        self.r_wheel = rospy.Publisher('/car/front_right_velocity_controller/command', Float64, queue_size=10)
        self.l_wheel = rospy.Publisher('/car/front_left_velocity_controller/command', Float64, queue_size=10)
        rospy.loginfo("Control nodes initialized")

        # Imposta la frequenza di pubblicazione a 10 Hz
        rate = rospy.Rate(10) # 10 Hz
        msg = Float64()

        while not rospy.is_shutdown():            

            msg.data = 3
            rospy.loginfo(f"Publishing velocity command: {msg.data}")
            self.r_wheel.publish(msg)
            
            msg.data = 1
            rospy.loginfo(f"Publishing velocity command: {msg.data}")
            self.r_wheel.publish(msg)
            
            # Dorme per mantenere il loop alla frequenza desiderata
            rate.sleep()
        
        #self.prova()
        
        self.sub = rospy.Subscriber("error", Float32, self.prova)

        rospy.loginfo("Error subscribed")
        return
    
    def prova(self):

        msg.data = 3
        rospy.loginfo(f"Publishing velocity command: {msg.data}")
        self.r_wheel.publish(msg)



        msg = std_msgs.msg.Float64()
        msg.data = 1.0
        rospy.loginfo("pubblico l")
        self.l_wheel.publish(msg)
        rospy.loginfo("pubblicato l")
        msg.data = 3.0
        self.r_wheel.publish(msg)
        rospy.loginfo("pubblicato r")
    


    def error_update(self, msg):

        '''error_data = msg
        current_time = time.time()

        if (current_time==0):
            return

        dt = current_time - self.time
        self.time = current_time
        # Compute PID output
        P = self.P_value * error_data
        I = self.I_value * self.accumulated_integral
        D = self.D_value * (error_data - self.prev_error) / dt
        control = P + I + D
    '''
        if self.thrust < MAX_VELOCITY:
            self.thrust += ADD_VELOCITY

        #right_velocity = self.thrust - TURNING * control
        #left_velocity = self.thrust + TURNING * control'''
        self.car_command(self.thrust, self.thrust)
    

    def car_command(self, right_velocity, left_velocity):
        msg = std_msgs.msg.Float64()
        msg.data = left_velocity
        self.l_wheel.pub(msg)
        msg.data = right_velocity
        self.r_wheel.pub(msg)
        rospy.loginfo("Command published")

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