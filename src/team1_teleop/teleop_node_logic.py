"""
A teleop node that has a timer, a publisher and a subscriber. The publisher publshes
the robot's arm position, and the subscribers receives arm move commands.
"""
import rospy
import tf2_ros
import tf2_geometry_msgs
from std_msgs.msg import Float64, String
from nav_msgs.msg import Odometry
from team1_teleop.msg import SavePose, SavedPoses
from geometry_msgs.msg import PoseStamped, PointStamped
import os
import pickle

import hello_helpers.hello_misc as hm
from hello_helpers.gripper_conversion import GripperConversion


class TeleopNode(hm.HelloNode):

    def __init__(self):
        hm.HelloNode.__init__(self)
        self.gc = GripperConversion()
        
    def main(self, node_name, node_topic_namespace, wait_for_first_pointcloud=False):
        super().main(node_name, node_topic_namespace, wait_for_first_pointcloud)

        # string pose_name -> (string frame_id, Pose robot_2d_loc, Pose walker_2d_loc)
        self.saved_poses_file = rospy.get_param("~/saved_poses", "/home/team1/catkin_ws/src/team1_teleop/saved_poses/saved_poses.pkl")
        if os.path.exists(self.saved_poses_file):
            with open(self.saved_poses_file, "rb") as file:
                self.saved_poses = pickle.load(file)
        else:
            self.saved_poses = {}
            self.saved_poses_file = "/home/team1/catkin_ws/src/team1_teleop/saved_poses/saved_poses.pkl"

        self.pub_saved_poses = rospy.Publisher('saved_pos_list', SavedPoses, queue_size=10)
        self.sub_save_pose = rospy.Subscriber('save_pose_cmd', SavePose, self.save_pose_callback)

        self.front_end_timer = rospy.Timer(rospy.Duration(0.5), self.front_end_timer_callback)
        self.aruco_timer = rospy.Timer(rospy.Duration(0.1), self.aruco_callback)
        self.curr_pos = None
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        # keep track of user's input (when they want to navigate the walker)
        self.pose_in_coordframe = rospy.Subscriber('pose_in_cf_cmd', String, self.pose_in_coordframe_callback)
        # the topic should be the one for nav algo to receive the pose
        self.walker_nav_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)

        """ Receives input from the user to grab the walker """
        self.gripper_nav_pub = rospy.Publisher('/clicked_point', PointStamped, queue_size=10)

        """ HARD CODE GRIPPER POSITION IN RELATION TO ARUCO TAG """
        self.gripper_pos_3d = PoseStamped()
        self.gripper_pos_3d.header.seq = 1
        self.gripper_pos_3d.header.frame_id = 'link_grasp_center'
        self.gripper_pos_3d.header.stamp = rospy.Time.now()


        """ 
        ***********************************************
        *** Simple joint subscribers and publishers ***
        ***********************************************
        """
        
        self.sub_head_pan = rospy.Subscriber('head_pan_cmd', Float64, self.move_head_pan_callback)
        self.sub_head_tilt = rospy.Subscriber('head_tilt_cmd', Float64, self.move_head_tilt_callback)
        self.sub_lift = rospy.Subscriber('lift_cmd', Float64, self.move_lift_callback)
        self.sub_arm = rospy.Subscriber('arm_cmd', Float64, self.move_arm_callback)
        self.sub_grip = rospy.Subscriber('grip_cmd', Float64, self.move_grip_callback)
        self.sub_wrist = rospy.Subscriber('wrist_cmd', Float64, self.move_wrist_callback)
        # self.sub_pose = rospy.Subscriber('pose_cmd', FrontEnd, self.move_pose_callback)

        self.sub_translate_base = rospy.Subscriber('translate_base_cmd', Float64, self.translate_base_callback, queue_size=1)
        # self.sub_rotate_base = rospy.Subscriber('rotate_base_cmd', Float64, self.rotate_base_callback, queue_size=1)
        # self.sub_stop_base = rospy.Subscriber('stop_base_cmd', Float64, self.stop_base_callback)

        rospy.loginfo("Node initialized")
    
    
    def save_pose_callback(self, msg: SavePose):
        if msg.pose_type == 0:
            source_frame = 'odom'
        elif msg.pose_type == 1:
            source_frame = 'link_grasp_center'
        else:
            raise ValueError(f"Invalid pose type recived in message {msg}")
        try:
            trans = self.tf2_buffer.lookup_transform(msg.pose_frame_id, source_frame, rospy.Time.now(), rospy.Duration(1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print(e)
            rospy.logwarn("Cannot transform saved pose")
            return
        rospy.loginfo("saved pose: " + msg.pose_name)
        if msg.pose_type == 0:
            pose = tf2_geometry_msgs.do_transform_pose(self.curr_pos, trans)
        else:
            pose = tf2_geometry_msgs.do_transform_pose(self.gripper_pos_3d, trans)
        self.saved_poses[msg.pose_name] = (msg.pose_frame_id, msg.pose_type, pose)

    def front_end_timer_callback(self, timer):
        msg = SavedPoses()
        msg.pose_names = list(self.saved_poses.keys())
        msg.pose_types = list(map(lambda v: v[1], self.saved_poses.values()))
        self.pub_saved_poses.publish(msg)
    
    def odom_callback(self, msg: Odometry):
        self.curr_pos = msg.pose
    
    def aruco_callback(self, timer):
        self.gripper_pos_3d.header.stamp = rospy.Time.now()

    def pose_in_coordframe_callback(self, msg):
        frame_id, pose_type, pose = self.saved_poses[msg.data]
        try:
            trans = self.tf2_buffer.lookup_transform("map", frame_id, rospy.Time.now(), rospy.Duration(1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Cannot get tf in map frame from the given coordinate frame")
            return
        rospy.loginfo("Sending goal message to funmap stack")
        if pose_type == 0:
            robot_pose_out = tf2_geometry_msgs.do_transform_pose(pose, trans)
            self.walker_nav_pub.publish(robot_pose_out)
        else:
            point = tf2_geometry_msgs.PointStamped()
            point.header = pose.header
            point.point.x = pose.pose.position.x
            point.point.y = pose.pose.position.y
            point.point.z = pose.pose.position.z
            gripper_point_out = tf2_geometry_msgs.do_transform_point(point, trans)
            self.gripper_nav_pub.publish(gripper_point_out)

    """
    ***********************************
    *** Simple move joint callbacks *** 
    ***********************************
    """

    def move_head_pan_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "Head pan move command received %f", data.data)
        pose = {"joint_head_pan": data.data}
        self.move_to_pose(pose)

    def move_head_tilt_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "Head tilt move command received %f", data.data)
        pose = {"joint_head_tilt": data.data}
        self.move_to_pose(pose)

    def move_lift_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "lift move command received %f", data.data)
        pose = {"joint_lift": data.data}
        self.move_to_pose(pose)

    def move_arm_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "Arm move command received %f", data.data)
        pose = {"joint_arm": data.data}
        self.move_to_pose(pose)

    def move_wrist_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "Wrist move command received %f", data.data)
        pose = {"joint_wrist_yaw": data.data}
        self.move_to_pose(pose)
    
    def move_grip_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "Gripper move command received %f", data.data)
        " investigate gripper_conversion.py "
        " ['joint_gripper_finger_left', 'joint_gripper_finger_right', 'gripper_aperture'] "
        apt = self.gc.finger_rad_to_aperture(data.data)
        pose = {"gripper_aperture": apt}
        self.move_to_pose(pose)

    def translate_base_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "Base command received %f", data.data)
          
    
    # def rotate_base_callback(self, data):
    #     rospy.loginfo(rospy.get_caller_id() + "Rotate command received %f", data.data)
    #     self.base.set_rotational_velocity(v_r=data.data)
    #     self.robot.push_command()

    # def stop_base_callback(self, data):
    #     rospy.loginfo(rospy.get_caller_id() + "stop command received %f", data.data)
    #     self.base.set_translate_velocity(v_m=data.data)
    #     self.base.set_rotational_velocity(v_r=data.data)
    #     self.robot.push_command()

    # def move_pose_callback(self, msg):
    #     """ Given a Front End msg move every joint of the robot """
    #     rospy.loginfo(msg)

    #     pose = {
    #         "joint_head_pan"        : msg.head_pan,
    #         "joint_head_tilt"       : msg.head_tilt,
    #         "joint_lift"            : msg.lift,
    #         "joint_arm"             : msg.arm,
    #         "joint_wrist_yaw"       : msg.wrist,
    #         "joint_stretch_gripper" : self.gripper.world_rad_to_pct(msg.grip)
    #         }
    #     self.move_to_pose(pose)

    def shutdown_hook(self):
        rospy.loginfo("Shutting down TeleopNode")
        if len(self.saved_poses):
            with open(self.saved_poses_file, "wb") as file:
                pickle.dump(self.saved_poses, file)
        