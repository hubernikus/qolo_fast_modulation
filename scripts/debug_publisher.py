"""
Additional Messages to simplify debugging
"""
import rospy 
from geometry_msgs.msg import PoseStamped, TwistStamped

class DebugPublisher:
    def __init__(self, main_controller):
        self.main_controller = main_controller
        
        self.publisher_vel_init = rospy.Publisher(
            'qolo/vel_init', TwistStamped, queue_size=1)

        self.publisher_vel_mod = rospy.Publisher(
            'qolo/vel_modul', TwistStamped, queue_size=1)

        self.publisher_reference = rospy.Publisher(
            'qolo/reference', TwistStamped, queue_size=1)


    def update_step(self, initial_velocity, modulated_velocity,
                        msg_time=None):
        # self.publish_twist_message(command_linear, command_angular)
        self.publish_twist(
            initial_velocity, msg_time=msg_time, publisher_handle=self.publisher_vel_init)
        
        self.publish_twist(
            modulated_velocity, msg_time=msg_time, publisher_handle=self.publisher_vel_mod)
        
        self.publish_twist(
            self.main_controller.fast_avoider.reference_direction,
            msg_time=msg_time, publisher_handle=self.publisher_reference)

    # def publish_twist_message(self, command_linear, command_angular):
        # msg = TwistStamped()
        # msg.header.stamp = rospy.Time.now()
        # msg.twist.linear.x = command_linear
        # msg.twist.angular.z = command_angular
        # self.publisher_twist.publish(msg)

    def publish_twist(self, vector, publisher_handle, msg_time=None):
        msg = TwistStamped()
        if msg_time is None:
            msg.header.stamp = rospy.Time.now()
        else:
            msg.header.stamp = msg_time
        msg.header.frame_id = "tf_qolo"

        msg.twist.linear.x = vector[0]
        msg.twist.linear.y = vector[1]

        publisher_handle.publish(msg)
