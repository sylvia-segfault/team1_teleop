"""
A teleop node that has a timer, a publisher and a subscriber. The publisher publshes
the robot's arm position, and the subscribers receives arm move commands.
"""
import rospy
import stretch_body.robot
import stretch_body.stretch_gripper
from std_msgs.msg import Float64, Float64MultiArray
from team1_teleop.msg import FrontEnd

class TeleopNode:

    def __init__(self):
        self.robot = stretch_body.robot.Robot()
        self.gripper = stretch_body.stretch_gripper.StretchGripper()
        self.robot.startup()
        self.head = self.robot.head
        self.lift = self.robot.lift
        self.arm = self.robot.arm
        self.end_of_arm = self.robot.end_of_arm
        self.base = self.robot.base

        self.pub_head_pan = rospy.Publisher('head_pan_pos', Float64, queue_size=10)
        self.pub_head_tilt = rospy.Publisher('head_tilt_pos', Float64, queue_size=10)
        self.pub_lift = rospy.Publisher('lift_pos', Float64, queue_size=10)
        self.pub_arm = rospy.Publisher('arm_pos', Float64, queue_size=10)
        self.pub_grip = rospy.Publisher('grip_pos', Float64, queue_size=10)
        self.pub_wrist = rospy.Publisher('wrist_pos', Float64, queue_size=10)
        
        self.sub_head_pan = rospy.Subscriber('head_pan_cmd', Float64, self.move_head_pan_callback)
        self.sub_head_tilt = rospy.Subscriber('head_tilt_cmd', Float64, self.move_head_tilt_callback)
        self.sub_lift = rospy.Subscriber('lift_cmd', Float64, self.move_lift_callback)
        self.sub_arm = rospy.Subscriber('arm_cmd', Float64, self.move_arm_callback)
        self.sub_grip = rospy.Subscriber('grip_cmd', Float64, self.move_grip_callback)
        self.sub_wrist = rospy.Subscriber('wrist_cmd', Float64, self.move_wrist_callback)
        self.sub_pose = rospy.Subscriber('pose_cmd', FrontEnd, self.move_pose_callback)

        self.sub_translate_base = rospy.Subscriber('translate_base_cmd', Float64, self.translate_base_callback, queue_size=1)
        self.sub_rotate_base = rospy.Subscriber('rotate_base_cmd', Float64, self.rotate_base_callback, queue_size=1)
        self.sub_stop_base = rospy.Subscriber('stop_base_cmd', Float64, self.stop_base_callback)

        self.set_motion_limits
        
        self.timer = rospy.Timer(rospy.Duration(0.5), self.timer_callback)
        rospy.loginfo("Node initialized")
    
    def set_motion_limits(self):
        self.lift.set_soft_motion_limit_min(0.25)
        self.lift.set_soft_motion_limit_max(1)

        self.arm.set_soft_motion_limit_min(0.0)
        self.arm.set_soft_motion_limit_max(1)
    
    def move_pose_callback(self, msg):
        rospy.loginfo(msg)
        self.robot.lift.move_to(msg.lift)
        self.robot.arm.move_to(msg.arm)
        self.end_of_arm.move_to('stretch_gripper', self.gripper.world_rad_to_pct(msg.grip))
        self.end_of_arm.move_to('wrist_yaw', msg.wrist)
        self.head.move_to('head_pan', msg.head_pan)
        self.head.move_to('head_tilt', msg.head_tilt)
        
        self.robot.push_command()

        self.lift.wait_until_at_setpoint()
        self.arm.wait_until_at_setpoint()
        self.end_of_arm.get_joint('stretch_gripper').wait_until_at_setpoint()
        self.end_of_arm.get_joint('wrist_yaw').wait_until_at_setpoint()
        self.head.get_joint('head_pan').wait_until_at_setpoint()
        self.head.get_joint('head_tilt').wait_until_at_setpoint()

    def move_head_pan_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "Head pan move command received %f", data.data)
        self.head.move_to('head_pan', data.data)
        self.robot.push_command()
        self.head.get_joint('head_pan').wait_until_at_setpoint()

    def move_head_tilt_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "Head tilt move command received %f", data.data)
        self.head.move_to('head_tilt', data.data)
        self.robot.push_command()
        self.head.get_joint('head_tilt').wait_until_at_setpoint()    

    def move_lift_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "lift move command received %f", data.data)
        self.robot.lift.move_to(data.data)
        self.robot.push_command()
        self.robot.lift.wait_until_at_setpoint()

    def move_arm_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "Arm move command received %f", data.data)
        self.robot.arm.move_to(data.data)
        self.robot.push_command()
        self.robot.arm.wait_until_at_setpoint()

    def move_wrist_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "Wrist move command received %f", data.data)
        self.end_of_arm.move_to('wrist_yaw', data.data)
        self.robot.push_command()
        self.end_of_arm.get_joint('wrist_yaw').wait_until_at_setpoint()
    
    def move_grip_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "Gripper move command received %f", data.data)
        pct_unit = self.gripper.world_rad_to_pct(data.data)
        self.end_of_arm.move_to('stretch_gripper', pct_unit)
        self.robot.push_command()
        self.end_of_arm.get_joint('stretch_gripper').wait_until_at_setpoint()

    def translate_base_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "Base command received %f", data.data)
        self.base.set_translate_velocity(v_m=data.data)
        self.robot.push_command()    
    
    def rotate_base_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "Rotate command received %f", data.data)
        self.base.set_rotational_velocity(v_r=data.data)
        self.robot.push_command()

    def stop_base_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "stop command received %f", data.data)
        self.base.set_translate_velocity(v_m=data.data)
        self.base.set_rotational_velocity(v_r=data.data)
        self.robot.push_command()

    def timer_callback(self, timer):
        lift_pos = self.robot.lift.status['pos']
        arm_pos = self.robot.arm.status['pos']
        grip_pos = self.robot.end_of_arm.status['stretch_gripper']['pos']
        wrist_pos = self.robot.end_of_arm.status['wrist_yaw']['pos']
        head_pan = self.robot.head.status['head_pan']['pos']
        head_tilt = self.robot.head.status['head_tilt']['pos']

        # rospy.loginfo("Publishing lift {}, arm {}, gripper {}, wrist {}, head pan {}, head tilt {}". format(lift_pos, arm_pos, grip_pos, wrist_pos, head_pan, head_tilt))
        self.pub_lift.publish(lift_pos)
        self.pub_arm.publish(arm_pos)
        self.pub_grip.publish(grip_pos)
        self.pub_wrist.publish(wrist_pos)
        self.pub_head_pan.publish(head_pan)
        self.pub_head_tilt.publish(head_tilt)
    
    def shutdown_hook(self):
        rospy.loginfo("Shutting down TeleopNode")
        self.robot.stop()

        