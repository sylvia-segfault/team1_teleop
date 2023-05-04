"""
A teleop node that has a timer, a publisher and a subscriber. The publisher publshes
the robot's arm position, and the subscribers receives arm move commands.
"""
import rospy
import stretch_body.robot
from std_msgs.msg import Float64, Float64MultiArray

class TeleopNode:

    def __init__(self):
        self.robot = stretch_body.robot.Robot()
        self.robot.startup()
        self.head = self.robot.head
        self.lift = self.robot.lift
        self.arm = self.robot.arm
        self.end_of_arm = self.robot.end_of_arm
        self.base = self.robot.base

        self.lift.set_soft_motion_limit_min(0.25)
        self.lift.set_soft_motion_limit_max(1)

        self.arm.set_soft_motion_limit_min(0.0)
        self.arm.set_soft_motion_limit_max(1)

        self.arm.wait_until_at_setpoint()
        self.pub_lift = rospy.Publisher('lift_pos', Float64, queue_size=10)
        self.pub_arm = rospy.Publisher('arm_pos', Float64, queue_size=10)
        self.pub_grip = rospy.Publisher('grip_pos', Float64, queue_size=10)
        self.pub_head_pan = rospy.Publisher('head_pan_pos', Float64, queue_size=10)
        self.pub_head_tilt = rospy.Publisher('head_tilt_pos', Float64, queue_size=10)
        self.pub_wrist = rospy.Publisher('wrist_pos', Float64, queue_size=10)

        self.sub_arm = rospy.Subscriber('arm_move_commands', Float64, self.move_arm_callback)
        self.sub_lift = rospy.Subscriber('lift_move_commands', Float64, self.move_lift_callback)
        self.sub_pose = rospy.Subscriber('pose_command', Float64MultiArray, self.move_pose_callback)
        self.timer = rospy.Timer(rospy.Duration(0.5), self.timer_callback)
        
    def starting_pose(self):
        pass


    def move_arm_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "Arm move command received %f", data.data)
        cur_pos = self.robot.arm.status['pos']
        new_pos = cur_pos + data.data
        self.robot.arm.move_to(new_pos)
        self.robot.push_command()
        self.robot.arm.wait_until_at_setpoint()

    def move_lift_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "lift move command received %f", data.data)
        cur_pos = self.robot.lift.status['pos']
        new_pos = cur_pos + data.data
        self.robot.lift.move_to(new_pos)
        self.robot.push_command()
        self.robot.lift.wait_until_at_setpoint()
    
    def move_pose_callback(self, data):
        # rospy.loginfo(rospy.get_caller_id() + "Post command received %f", data.data)

        self.robot.lift.move_to(data.data[0])
        print(data.data)
        self.robot.arm.move_to(data.data[1])

        
        self.end_of_arm.move_to('stretch_gripper', data.data[2])
        self.end_of_arm.move_to('wrist_yaw', data.data[3])
        self.head.move_to('head_pan', data.data[4])
        self.head.move_to('head_tilt', data.data[5])
        
        self.robot.push_command()
        self.robot.lift.wait_until_at_setpoint()
        self.robot.arm.wait_until_at_setpoint()
        self.robot.end_of_arm.get_joint('stretch_gripper').wait_until_at_setpoint()
        self.robot.end_of_arm.get_joint('wrist_yaw').wait_until_at_setpoint()
        self.head.get_joint('head_pan').wait_until_at_setpoint()
        self.head.get_joint('head_tilt').wait_until_at_setpoint()
        


    def pan_head_callback(self, data):
        pass

    def tilt_head_callback(self, data):
        pass
    
    def timer_callback(self, timer):
        lift_pos = self.robot.lift.status['pos']
        arm_pos = self.robot.arm.status['pos']
        grip_pos = self.robot.end_of_arm.status['stretch_gripper']['pos']
        wrist_pos = self.robot.end_of_arm.status['wrist_yaw']['pos']
        head_pan = self.robot.head.status['head_pan']['pos']
        head_tilt = self.robot.head.status['head_tilt']['pos']

        # rospy.loginfo("Publishing lift position {}".format(lift_pos))
        # rospy.loginfo("Publishing arm position {}".format(arm_pos))
        # rospy.loginfo("Publishing gripper position {}".format(grip_pos))
        # rospy.loginfo("Publishing wrist position {}".format(wrist_pos))
        # rospy.loginfo("Publishing head pan position {}".format(head_pan))
        # rospy.loginfo("Publishing head tilt position {}".format(head_tilt))
        self.pub_lift.publish(lift_pos)
        self.pub_arm.publish(arm_pos)
        self.pub_grip.publish(grip_pos)
        self.pub_wrist.publish(wrist_pos)
        self.pub_head_pan.publish(head_pan)
        self.pub_head_tilt.publish(head_tilt)
    
    def shutdown_hook(self):
        rospy.loginfo("Shutting down TeleopNode")
        self.robot.stop()

        