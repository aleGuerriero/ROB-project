


import rospy
from std_msgs.msg import Float64


class Plotter:
    """
    Class for plotting using plotjugger
    """
    def __init__(
            self,
            debug: bool = False
    ) -> None:
        
        self.p_errx = rospy.Publisher("/err_x", Float64, queue_size=1)
        self.p_0 = rospy.Publisher("/0", Float64, queue_size=1)
        self.p_0.publish(0)
        self.p_theta = rospy.Publisher("/err_theta", Float64, queue_size=1)
        self.p_rv = rospy.Publisher("/right_velocity", Float64, queue_size=1)
        self.p_lv = rospy.Publisher("/left_velocity", Float64, queue_size=1)

    def plot_errors(self, errx,errtheta):
        self.p_errx.publish(errx)
        self.p_theta.publish(errtheta)
        return
    
    def plot_velocities(self, rv, lv):
        self.p_rv.publish(rv)
        self.p_lv.publish(lv)
        return
