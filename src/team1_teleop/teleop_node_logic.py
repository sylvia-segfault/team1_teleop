"""
A teleop node that has a timer, a publisher and a subscriber. The publisher publshes
the robot's arm position, and the subscribers receives arm move commands.
"""
import rospy
import stretch_body.robot
from std_msgs.msg import Float64

class TeleopNode:

    def __init__(self):
        self.robot = stretch_body.robot.Robot()
        self.robot.startup()
        self.arm_pos = 0.0
        self.robot.arm.move_to(0.0)
        self.robot.push_command()
        self.robot.arm.wait_until_at_setpoint()
        self.pub = rospy.Publisher('arm_pos', Float64, queue_size=10)
        self.sub = rospy.Subscriber('arm_move_commands', Float64, self.move_callback)
        self.timer = rospy.Timer(rospy.Duration(0.5), self.timer_callback)
        

    def move_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "Arm move command received %f", data.data)
        # self.robot.base.translate_by(x_m=data.data)
        self.arm_pos += data.data
        if self.arm_pos < 0:
            self.arm_pos = 0
        self.robot.arm.move_to(self.arm_pos)
        self.robot.push_command()
        self.robot.arm.wait_until_at_setpoint()
    
    def timer_callback(self, timer):
        pos = self.robot.arm.status['pos']
        rospy.loginfo("Publishing arm position {}".format(pos))
        self.pub.publish(pos)
    
    def shutdown_hook(self):
        rospy.loginfo("Shutting down TeleopNode")
        self.robot.stop()

        