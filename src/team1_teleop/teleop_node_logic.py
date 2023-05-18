"""
A teleop node that has a timer, a publisher and a subscriber. The publisher publshes
the robot's arm position, and the subscribers receives arm move commands.
"""
import rospy
import tf2_ros
import tf2_geometry_msgs
import stretch_body.robot
import stretch_body.stretch_gripper
from std_msgs.msg import Float64, String
from nav_msgs.msg import Odometry
from team1_teleop.msg import FrontEnd, SavePose
from geometry_msgs.msg import PoseStamped, PointStamped
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
import os
import pickle

import hello_helpers.hello_misc as hm
import stretch_funmap.navigate as nv
import stretch_funmap.manipulation_planning as mp

class TeleopNode(hm.HelloNode):

    def __init__(self):
        hm.HelloNode.__init__(self)
        
    def main(self, node_name, node_topic_namespace, wait_for_first_pointcloud=False):
        super().main(node_name, node_topic_namespace, wait_for_first_pointcloud)
        # self.robot = stretch_body.robot.Robot()
        # self.gripper = stretch_body.stretch_gripper.StretchGripper()
        self.robot.startup()
        self.head = self.robot.head
        self.lift = self.robot.lift
        self.arm = self.robot.arm
        self.end_of_arm = self.robot.end_of_arm
        self.base = self.robot.base

        # string pose_name -> (string frame_id, Pose robot_2d_loc, Pose walker_2d_loc)
        self.saved_poses_file = rospy.get_param("~/saved_poses", "/home/team1/catkin_ws/src/team1_teleop/saved_poses/saved_poses.pkl")
        if os.path.exists(self.saved_poses_file):
            with open(self.saved_poses_file, "rb") as file:
                self.saved_poses = pickle.load(file)
        else:
            self.saved_poses = {}
            self.saved_poses_file = "/home/team1/catkin_ws/src/team1_teleop/saved_poses/saved_poses.pkl"

        self.pub_saved_poses = rospy.Publisher('saved_pos_list', String, queue_size=10)
        self.sub_save_pose = rospy.Subscriber('save_pose_cmd', SavePose, self.save_pose_callback)

        self.front_end_timer = rospy.Timer(rospy.Duration(0.5), self.front_end_timer_callback)
        self.aruco_timer = rospy.Timer(rospy.Duration(0.1), self.aruco_callback)
        
        # walker 2d pose
        self.walker_center = None
        self.curr_pos = None
        self.tf2_buffer = tf2_ros.Buffer()
        # tf2_ros.TransformListener(self.tf2_buffer)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        # keep track of user's input (when they want to navigate the walker)
        self.pose_in_coordframe = rospy.Subscriber('pose_in_cf_cmd', String, self.pose_in_coordframe_callback)
        # the topic should be the one for nav algo to receive the pose
        self.walker_nav_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)

        """ Receives input from the user to grab the walker """
        self.walker_grab_sub = rospy.Subscriber('walker_grab_cmd', String, self.walker_grab_callback)
        self.gripper_nav_pub = rospy.Publisher('/clicked_point', PointStamped, queue_size=10)

        
        """ HARD CODE WALKER POSITON IN RELATION TO ARUCO TAG """

        self.walker_pos_3d = PoseStamped()
        self.walker_pos_3d.header.seq = 1
        self.walker_pos_3d.header.frame_id = 'walker_center'
        self.walker_pos_3d.header.stamp = rospy.Time.now()
        self.walker_pos_3d.pose.position.x = 0
        self.walker_pos_3d.pose.position.y = 0
        self.walker_pos_3d.pose.position.z = 0
        self.walker_pos_3d.pose.orientation.x = 0
        self.walker_pos_3d.pose.orientation.y = -0.7068252
        self.walker_pos_3d.pose.orientation.z = 0
        self.walker_pos_3d.pose.orientation.w = 0.7073883

        """ 
        ***********************************************
        *** Simple joint subscribers and publishers ***
        ***********************************************
        """
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

        self.set_motion_limits()
        rospy.loginfo("Node initialized")
    
    def set_motion_limits(self):
        self.lift.set_soft_motion_limit_min(0.25)
        self.lift.set_soft_motion_limit_max(1)

        self.arm.set_soft_motion_limit_min(0.0)
        self.arm.set_soft_motion_limit_max(1)
    
    
    def save_pose_callback(self, msg: SavePose):
        if self.curr_pos is None or self.walker_center is None:
            # TODO: This should be a service which can indicate that it failed
            rospy.logwarn("save pose callback returned early: " 
                          + self.curr_pos + ", " + self.walker_center)
            return
        try:
            # self.walker_center is the walker pose in the 2d frame of the map
            # Note that source frame in lookup_transform 'walker_center' should 
            # instead be whatever frame self.walker_center is saved in ("map")
            walker_trans = self.tf2_buffer.lookup_transform(msg.pose_frame_id, 'walker_center', self.walker_pos_3d.header.stamp, rospy.Duration(1))
            odom_trans = self.tf2_buffer.lookup_transform(msg.pose_frame_id, 'odom', self.walker_pos_3d.header.stamp, rospy.Duration(1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Cannot transform saved pose")
            return
        rospy.loginfo("saved pose: " + msg.pose_name)
        walker_pos = tf2_geometry_msgs.do_transform_pose(self.walker_center, walker_trans)
        odom_pose = tf2_geometry_msgs.do_transform_pose(self.curr_pos, odom_trans)
        self.saved_poses[msg.pose_name] = (msg.pose_frame_id, odom_pose, walker_pos)

    def front_end_timer_callback(self, timer):
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
        self.pub_saved_poses.publish(','.join(self.saved_poses.keys()))
        # rospy.loginfo("sent poses: " + ','.join(self.saved_poses.keys()))
    
    def odom_callback(self, msg: Odometry):
        # try:
        #     trans = self.tf2_buffer.lookup_transform('map', "odom", rospy.Time.now(), rospy.Duration(1))
        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #     rospy.logwarn("Could not transform odom into map frame.")
        #     self.walker_center = None
        #     return
        # curr_pos_2d = tf2_geometry_msgs.do_transform_pose(msg.pose, trans)
        # curr_pos_2d.pose.position.z = 0
        # q = curr_pos_2d.pose.orientation
        # _, _, y = tf_conversions.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        # new_quat = tf_conversions.transformations.quaternion_from_euler(0, 0, y)
        # curr_pos_2d.pose.orientation.x = new_quat[0]
        # curr_pos_2d.pose.orientation.y = new_quat[1]
        # curr_pos_2d.pose.orientation.z = new_quat[2]
        # curr_pos_2d.pose.orientation.w = new_quat[3]
        # self.curr_pos = curr_pos_2d
        self.curr_pos = msg.pose
    
    def aruco_callback(self, timer):
        self.walker_pos_3d.header.stamp = rospy.Time.now()
        # try:
        #     trans = self.tf2_buffer.lookup_transform('map', 'walker_center', rospy.Time.now(), rospy.Duration(1))
        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        #     rospy.logwarn("Walker center not found.")
        #     self.walker_center = None
        #     return
        # walker_pos_2d = tf2_geometry_msgs.do_transform_pose(self.walker_pos_3d, trans)
        # walker_pos_2d.pose.position.z = 0
        # q = walker_pos_2d.pose.orientation
        # _, _, y = tf_conversions.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        # new_quat = tf_conversions.transformations.quaternion_from_euler(0, 0, y)
        # walker_pos_2d.pose.orientation.x = new_quat[0]
        # walker_pos_2d.pose.orientation.y = new_quat[1]
        # walker_pos_2d.pose.orientation.z = new_quat[2]
        # walker_pos_2d.pose.orientation.w = new_quat[3]
        # self.walker_center = walker_pos_2d

    def pose_in_coordframe_callback(self, msg):
        frame_id, robot_pose, _ = self.saved_poses[msg.data]
        try:
            trans = self.tf2_buffer.lookup_transform("map", frame_id, self.walker_pos_3d.header.stamp, rospy.Duration(1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Cannot get tf in map frame from the given coordinate frame")
            return
        rospy.loginfo("Sending goal message to stretch_navigation stack")
        robot_pose_out = tf2_geometry_msgs.do_transform_pose(robot_pose, trans)
        self.walker_nav_pub.publish(robot_pose_out)

    def walker_grab_callback(self, msg):
        # scan the environment around the walker and create a map

        # locate the walker on FUNMAP
        # the FUNMAP might need to be ran separately from the nav stack, so that the map frame
        # corresponds with FUNMAP
        try:
            trans = self.tf2_buffer.lookup_transform("map", self.walker_pos_3d.header.frame_id, self.walker_pos_3d.header.stamp, rospy.Duration(1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Cannot get tf of the walker in FUNMAP")
            return
        rospy.loginfo("Sending point message to FUNMAP")
        gripper_point_out = tf2_geometry_msgs.do_transform_point()

        # publish target click point in FUNMAP (with a radius of walker position)
        pass

    """
    ***********************************
    *** Simple move joint callbacks *** 
    ***********************************
    """

    def move_head_pan_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "Head pan move command received %f", data.data)
        self.head.move_to('head_pan', data.data)
        self.robot.push_command()
        self.head.get_joint('head_pan').wait_until_at_setpoint()

        # pose = {"joint_head_pan": data.data}
        # self.move_to_pose(pose)

    def move_head_tilt_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "Head tilt move command received %f", data.data)
        # pose = {"joint_head_tilt": data.data}
        # self.move_to_pose(pose)
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
        rospy.logwarn("HERE")
        rospy.loginfo(rospy.get_caller_id() + "Wrist move command received %f", data.data)
        # self.end_of_arm.move_to('wrist_yaw', data.data)
        # self.robot.push_command()
        # self.end_of_arm.get_joint('wrist_yaw').wait_until_at_setpoint()
        pose = {"wrist_extension": data.data}
        self.move_to_pose(pose)
    
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

    def move_pose_callback(self, msg):
        """ Given a Front End msg move every joint of the robot """
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

    def shutdown_hook(self):
        rospy.loginfo("Shutting down TeleopNode")
        self.robot.stop()
        if len(self.saved_poses):
            with open(self.saved_poses_file, "wb") as file:
                pickle.dump(self.saved_poses, file)
        