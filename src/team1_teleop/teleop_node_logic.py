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
        #self.arm_pos = 0.0
        self.robot.arm.move_to(0.0)
        self.robot.push_command()
        self.robot.arm.wait_until_at_setpoint()
        self.pub = rospy.Publisher('lift_pos', Float64, queue_size=10)
        self.pub = rospy.Publisher('arm_pos', Float64, queue_size=10)
        self.pub = rospy.Publisher('grip_pos', Float64, queue_size=10)
        self.sub = rospy.Subscriber('arm_move_commands', Float64, self.move_arm_callback)
        self.sub = rospy.Subscriber('lift_move_commands', Float64, self.move_lift_callback)
        self.timer = rospy.Timer(rospy.Duration(0.5), self.timer_callback)
        

    def move_arm_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "Arm move command received %f", data.data)
        # self.robot.base.translate_by(x_m=data.data)

        arm_min = 0.0
        arm_max = 0.51
        T = 0.00001

        cur_pos = self.robot.arm.status['pos']
        new_pos = cur_pos + data.data
        if new_pos < arm_min:
            new_pos = arm_min
        elif new_pos > arm_max:
            new_pos = arm_max
        if (not (abs(new_pos-cur_pos) < T)):
          self.robot.arm.move_to(new_pos)
          self.robot.push_command()
          self.robot.arm.wait_until_at_setpoint()

    def move_lift_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "lift move command received %f", data.data)
        # self.robot.base.translate_by(x_m=data.data)
        lift_pos = self.robot.lift.status['pos'] + data.data
        # if self.arm_pos < 0:
        #     self.arm_pos = 0
        self.robot.lift.move_to(lift_pos)
        self.robot.push_command()
        self.robot.lift.wait_until_at_setpoint()
    
    def timer_callback(self, timer):
        lift_pos = self.robot.lift.status['pos']
        arm_pos = self.robot.arm.status['pos']
        grip_pos = 1

        rospy.loginfo("Publishing lift position {}".format(lift_pos))
        rospy.loginfo("Publishing arm position {}".format(arm_pos))
        rospy.loginfo("Publishing gripper position {}".format(grip_pos))
        self.pub.publish(lift_pos)
        self.pub.publish(arm_pos)
        self.pub.publish(grip_pos)
    
    def shutdown_hook(self):
        rospy.loginfo("Shutting down TeleopNode")
        self.robot.stop()

        