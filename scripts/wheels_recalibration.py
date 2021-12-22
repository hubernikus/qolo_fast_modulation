"""
Debug test - fast wheel spinner
"""
import rospy
import signal
import time

from std_msgs.msg import Float32MultiArray, MultiArrayDimension

class FastWheelSpinning():
    def __init__(self):
        rospy.init_node('wheel_callibrator')

        # Create ctrl-c handler
        signal.signal(signal.SIGINT, self.control_c_handler)

        self.loop_rate = 100
        self.rate = rospy.Rate(self.loop_rate) # Hz

        self.pub_qolo_command = rospy.Publisher(
            'qolo/remote_commands', Float32MultiArray, queue_size=1)

        # Shutdown variable
        self.shutdown_finished = False


    def publish_command(self, command_linear, command_angular):
        """  Command to QOLO motor [Real Implementation]. 
        Includes MASTER CHECK if linear/angular velocity reached limit
        """
        data_remote = Float32MultiArray()
        data_remote.layout.dim.append(MultiArrayDimension())
        data_remote.layout.dim[0].label = 'Trajectory Commands [V, W]'
        data_remote.layout.dim[0].size = 3
        data_remote.data = [0]*3

        # Use 'time' since 'rospy.time' is a too large float
        msg_time = round(time.perf_counter(), 4)

        data_remote.data = [msg_time, command_linear, command_angular]
        # warnings.warn("Not publishing here though..")
        self.pub_qolo_command.publish(data_remote)

    def run(self):
        while not rospy.is_shutdown():
            # Start with sleep for initialization & error in case of shutdown
            self.publish_command(1, 0)
            self.rate.sleep()

    def control_c_handler(self, sig, frame):
        """ User defined handling of ctrl-c"""
        print('\nCaught ctrl-c by user. Shutdown is initiated ...')
        self.shutdown()
        rospy.signal_shutdown('Caught ctrl-c by user. Shutdown is initiated ...')

    def shutdown(self):
        """ User defined shutdown command."""

        print("\nDoing shutdown.")
        if self.shutdown_finished:
            return

        # Published repeated zero velocity to ensure stand-still
        for ii in range(10):
            self.publish_command(0, 0)
            rospy.sleep(0.01) # ? why error...

        self.shutdown_finished = True
        print("\nShutdown successful.")


if (__name__) == "__main__":
    continue_str = input("[WARNING] This script will spin both wheels at a high rate. "
                         + "Make sure the gears are deactivated. \n"
                         + "          Do you want to continue [y/N]:\n")

    if continue_str=="y" or continue_str=="yes" or continue_str=="Y":
        print("Starting the spinning. Use Ctrl-C to exit.")
        FastWheelSpinning().run()
