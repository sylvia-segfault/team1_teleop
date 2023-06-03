"""
A teleop node that has a timer, a publisher and a subscriber. The publisher publshes
the robot's arm position, and the subscribers receives arm move commands.
"""
import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, String, Bool
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import tf_conversions
from team1_teleop.msg import SavePose, SavedPoses
from geometry_msgs.msg import PoseStamped
import os
import pickle
import math

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
        self.home = None
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        # keep track of user's input (when they want to navigate the walker)
        self.pose_in_coordframe = rospy.Subscriber('pose_in_cf_cmd', String, self.pose_in_coordframe_callback)
        # the topic should be the one for nav algo to receive the pose
        self.walker_nav_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)

        """ Receives input from the user to grab the walker """
        self.joint_states_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)

        """ Allows this node to move the base, mostly to get close enough to the walker """
        self.move_base_pub = rospy.Publisher("/stretch/cmd_vel", Twist, queue_size=10)

        """ Drive backwards to return the walker back to the robot's starting position """
        self.home_sub = rospy.Subscriber('/home', Bool, self.home_callback)

        """ HARD CODE GRIPPER POSITION IN RELATION TO ARUCO TAG """
        self.gripper_pos_3d = PoseStamped()
        self.gripper_pos_3d.header.seq = 1
        self.gripper_pos_3d.header.frame_id = 'link_grasp_center'
        self.gripper_pos_3d.header.stamp = rospy.Time.now()

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

        # test gripper poses
        self.test_target_pub = rospy.Publisher("/test_pose_target", PoseStamped, queue_size=10)
        self.test_cur_pub = rospy.Publisher("/test_pose_curr", PoseStamped, queue_size=10)

        self.cur_lift_pos = None
        self.cur_arm_pos = None

        self.odom_pos_3d = PoseStamped()
        self.odom_pos_3d.header.seq = 1
        self.odom_pos_3d.header.frame_id = 'odom'
        self.odom_pos_3d.header.stamp = rospy.Time.now()

        # TODO: delete
        self.test_pub1 = rospy.Publisher("/odom_actual", PoseStamped, queue_size=1)
        self.test_pub2 = rospy.Publisher("/odom_pose_vis", PoseStamped, queue_size=10)

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

        self.sub_translate_base = rospy.Subscriber('translate_base_cmd', Float64, self.translate_base_callback, queue_size=1)
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
        if self.home is None:
            rospy.loginfo("Setting home in map frame")
            stamped = PoseStamped()
            stamped.header = msg.header
            stamped.pose = msg.pose.pose

            # transform odom into the map frame
            try:
                trans = self.tf2_buffer.lookup_transform("map", "odom", rospy.Time.now(), rospy.Duration(1))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                print(e)
                rospy.logwarn("Cannot transform the odom frame into the map frame")
                return
            
            map_pose = tf2_geometry_msgs.do_transform_pose(stamped, trans)
            self.home = map_pose
        # TODO: Delete this stuff
        # self.test_pub2.publish(stamped)
    
    def aruco_callback(self, timer):
        self.gripper_pos_3d.header.stamp = rospy.Time.now()
        self.walker_pos_3d.header.stamp = rospy.Time.now()
        try:
            gripper_trans_after = self.tf2_buffer.lookup_transform('xy_walker_center', 'link_grasp_center', rospy.Time.now(), rospy.Duration(1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print(e)
            rospy.logwarn("Cannot transform the gripper frame into the specified coordinate frame")
            return
        cur_pose = tf2_geometry_msgs.do_transform_pose(self.gripper_pos_3d, gripper_trans_after)
        self.test_cur_pub.publish(cur_pose)

    def pose_in_coordframe_callback(self, msg):
        self.gripper_pos_3d.header.stamp = rospy.Time.now()
        self.walker_pos_3d.header.stamp = rospy.Time.now()
        frame_id, pose_type, pose = self.saved_poses[msg.data]
        rospy.loginfo(frame_id)
        pose.header.stamp = rospy.Time.now()
        rospy.loginfo("Sending goal message to navigation stack")
        if pose_type == 0:
            # call stretch navigation
            if "step1" in msg.data:
                camera_pose = {
                    "joint_head_pan": 0,
                    "joint_head_tilt": -0.5
                }
                self.move_to_pose(camera_pose)
                rospy.sleep(2)
            elif "step2" in msg.data:
                # -0.6
                #-0.9
                camera_pose = {
                    "joint_head_pan": -0.3,
                    "joint_head_tilt": -0.9
                }
                self.move_to_pose(camera_pose)
                rospy.sleep(2)
            try:
                trans = self.tf2_buffer.lookup_transform("map", frame_id, rospy.Time.now(), rospy.Duration(1))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logerr("Cannot get tf in map frame from the given coordinate frame")
                return
            robot_pose_out = tf2_geometry_msgs.do_transform_pose(pose, trans)
            self.walker_nav_pub.publish(robot_pose_out)
        else:
            # align the gripper
            camera_pose = {
                "joint_head_pan": -0.5,
                "joint_head_tilt": -0.9
            }
            self.move_to_pose(camera_pose)
            rospy.sleep(2)
            try:
                gripper_trans = self.tf2_buffer.lookup_transform(frame_id, 'link_grasp_center', rospy.Time.now(), rospy.Duration(1))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                print(e)
                rospy.logwarn("Cannot transform the gripper frame into the specified coordinate frame")
                return
            cur_pose = tf2_geometry_msgs.do_transform_pose(self.gripper_pos_3d, gripper_trans)
            rospy.loginfo('current x: ' + str(cur_pose.pose.position.x))
            rospy.loginfo('current y: ' + str(cur_pose.pose.position.y))
            rospy.loginfo('current z: ' + str(cur_pose.pose.position.z))

            apt = self.gc.finger_rad_to_aperture(0.2)
            self.move_to_pose({"gripper_aperture": apt})
            self.test_target_pub.publish(pose)
            
            """
            transform /odom into the specified frame
            convert it into r, p, y and compare with the goal's r, p, y
            either rotate right wheel (negative) or left wheel (positive)
            """
            self.odom_pos_3d.pose = self.curr_pos.pose
            base_trans = self.tf2_buffer.lookup_transform(frame_id, 'odom', rospy.Time.now(), rospy.Duration(1))
            base_pos = tf2_geometry_msgs.do_transform_pose(self.odom_pos_3d, base_trans)
            q_odom = base_pos.pose.orientation
            _, _, y_odom = tf_conversions.transformations.euler_from_quaternion([q_odom.x, q_odom.y, q_odom.z, q_odom.w])
            q_goal = pose.pose.orientation
            _, _, y_goal = tf_conversions.transformations.euler_from_quaternion([q_goal.x, q_goal.y, q_goal.z, q_goal.w])

            y_odom = self.normalize_angle(y_odom)
            y_goal = self.normalize_angle(y_goal)

            base_msg = Twist()
            base_thresh = 0.001
            odom_diff = y_goal - y_odom
            
            while abs(odom_diff) > base_thresh:
                # self.test_pub1.publish(y_odom)
                if y_odom < y_goal:
                    # turn left
                    base_msg.angular.z = 0.05
                    rospy.loginfo("incre")
                else:
                    base_msg.angular.z = -0.05
                    rospy.loginfo("decre")
                self.move_base_pub.publish(base_msg)
                self.odom_pos_3d.pose = self.curr_pos.pose
                base_trans = self.tf2_buffer.lookup_transform(frame_id, 'odom', rospy.Time.now(), rospy.Duration(1))
                base_pos = tf2_geometry_msgs.do_transform_pose(self.odom_pos_3d, base_trans)
                q_odom = base_pos.pose.orientation
                _, _, y_odom = tf_conversions.transformations.euler_from_quaternion([q_odom.x, q_odom.y, q_odom.z, q_odom.w])
                y_odom = self.normalize_angle(y_odom)
                odom_diff = y_goal - y_odom
                rospy.loginfo(f"Goal y:  {y_goal}")
                rospy.loginfo(f"Odom pose y: {y_odom}")
                rospy.loginfo(f"diff y: {odom_diff}")
                rospy.loginfo("*****************")

            # retract the arm, y becomes smaller by a similar amount
            goal_y = pose.pose.position.y
            y_threshold = 0.0005
            y_diff = cur_pose.pose.position.y - goal_y
            while abs(y_diff) > y_threshold:
                self.gripper_pos_3d.header.stamp = rospy.Time.now()
                self.walker_pos_3d.header.stamp = rospy.Time.now()
                # self.test_cur_pub.publish(cur_pose)
                sign = 1 if cur_pose.pose.position.y > goal_y else -1
                arm_pose = {"joint_arm": self.cur_arm_pos - sign * 0.01}
                self.move_to_pose(arm_pose)
                try:
                    gripper_trans_after = self.tf2_buffer.lookup_transform(frame_id, 'link_grasp_center', rospy.Time.now(), rospy.Duration(1))
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    print(e)
                    rospy.logwarn("Cannot transform the gripper frame into the specified coordinate frame")
                    return
                cur_pose = tf2_geometry_msgs.do_transform_pose(self.gripper_pos_3d, gripper_trans_after)
                rospy.loginfo(f"Cur_pose y:  {cur_pose.pose.position.y}")
                rospy.loginfo(f"Goal pose y: {goal_y}")
                rospy.loginfo(f"Cur_pose x:  {cur_pose.pose.position.x}")
                rospy.loginfo(f"Goal pose x: {pose.pose.position.x}")
                rospy.loginfo("*****************")
                y_diff = cur_pose.pose.position.y - goal_y
                    
            self.move_to_pose({"joint_wrist_yaw": 1.5})
            rospy.loginfo(y_diff)
            
            # lower the lift, z becomes smaller by a similar amount!
            goal_z = pose.pose.position.z
            z_threshold = 0.005
            z_diff = cur_pose.pose.position.z - goal_z
            while abs(z_diff) > z_threshold:
                self.gripper_pos_3d.header.stamp = rospy.Time.now()
                self.walker_pos_3d.header.stamp = rospy.Time.now()
                self.test_cur_pub.publish(cur_pose)
                sign = 1 if cur_pose.pose.position.z > goal_z else -1
                lift_pose = {"joint_lift": self.cur_lift_pos - sign * 0.01}
                self.move_to_pose(lift_pose)
                try:
                    gripper_trans_after = self.tf2_buffer.lookup_transform(frame_id, 'link_grasp_center', rospy.Time.now(), rospy.Duration(1))
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    print(e)
                    rospy.logwarn("Cannot transform the gripper frame into the specified coordinate frame")
                    return
                cur_pose = tf2_geometry_msgs.do_transform_pose(self.gripper_pos_3d, gripper_trans_after)
                rospy.loginfo(f"Cur_pose y:  {cur_pose.pose.position.y}")
                rospy.loginfo(f"Goal pose y: {goal_y}")
                rospy.loginfo(f"Cur_pose x:  {cur_pose.pose.position.x}")
                rospy.loginfo(f"Goal pose x: {pose.pose.position.x}")
                rospy.loginfo("*****************")
                z_diff = cur_pose.pose.position.z - goal_z

            rospy.loginfo("Attempting to adjust x")
            goal_x = pose.pose.position.x
            x_threshold = 0.02
            x_diff = cur_pose.pose.position.x - goal_x
            while abs(x_diff) > x_threshold:
                self.gripper_pos_3d.header.stamp = rospy.Time.now()
                self.walker_pos_3d.header.stamp = rospy.Time.now()
                self.test_cur_pub.publish(cur_pose)
                sign = 1 if cur_pose.pose.position.x > goal_x else -1
                base_msg = Twist()
                base_msg.linear.x = 0.05
                self.move_base_pub.publish(base_msg)
                rospy.sleep(1)
                try:
                    base_trans_after = self.tf2_buffer.lookup_transform(frame_id, "link_grasp_center", rospy.Time.now(), rospy.Duration(1))
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    print(e)
                    rospy.logwarn("Cannot transform the gripper frame into the specified coordinate frame, x loop")
                    return
                cur_pose = tf2_geometry_msgs.do_transform_pose(self.gripper_pos_3d, base_trans_after)
                rospy.loginfo(f"Cur_pose y:  {cur_pose.pose.position.y}")
                rospy.loginfo(f"Goal pose y: {goal_y}")
                rospy.loginfo(f"Cur_pose x:  {cur_pose.pose.position.x}")
                rospy.loginfo(f"Goal pose x: {pose.pose.position.x}")
                rospy.loginfo("*****************")
                x_diff = cur_pose.pose.position.x - goal_x


            apt = self.gc.finger_rad_to_aperture(-0.3)
            self.move_to_pose({"gripper_aperture": apt})

    def joint_state_callback(self, msg):
        self.cur_lift_pos, _, _ = hm.get_lift_state(msg)
        self.cur_arm_pos = msg.position[8] + msg.position[7] + msg.position[6] + msg.position[5]
    
    def home_callback(self, msg: Bool):
        if not msg.data:
            return
        
        self.odom_pos_3d.pose = self.curr_pos.pose
        self.test_pub1.publish(self.odom_pos_3d)
        try:
            odom_to_map_trans = self.tf2_buffer.lookup_transform("map", "odom", rospy.Time.now(), rospy.Duration(1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print(e)
            rospy.logwarn("Cannot transform the gripper frame into the specified coordinate frame")
            return
        odom_flip = tf2_geometry_msgs.do_transform_pose(self.odom_pos_3d, odom_to_map_trans)
        q = odom_flip.pose.orientation
        roll, pitch, yaw = tf_conversions.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        rospy.loginfo(f"r: {roll}, p: {pitch}, y: {yaw}")
        yaw += math.pi
        rospy.loginfo(yaw)
        new_quat = tf_conversions.transformations.quaternion_from_euler(0, 0, yaw)
        odom_flip.pose.orientation.x = new_quat[0]
        odom_flip.pose.orientation.y = new_quat[1]
        odom_flip.pose.orientation.z = new_quat[2]
        odom_flip.pose.orientation.w = new_quat[3]

        base_msg = Twist()

        # tan theta = y/x
        # theta = arctan(y/x)
        y_diff = abs(self.home.pose.position.y - odom_flip.pose.position.y)
        x_diff = abs(self.home.pose.position.x - odom_flip.pose.position.x)
        angle_needed = math.atan(y_diff / x_diff)
        dist = math.sqrt((odom_flip.pose.position.x - self.home.pose.position.x) ** 2 + (odom_flip.pose.position.y - self.home.pose.position.y) ** 2)

        turn_thresh = 0.1
        dist_thresh = 0.3
        while abs(yaw - angle_needed) > turn_thresh or dist > dist_thresh:
            self.test_target_pub.publish(self.home)
            rospy.loginfo(f"angle_needed: {angle_needed}, current yaw: {yaw}, distance: {dist}")
            if angle_needed > yaw:
                # turn left
                base_msg.angular.z = 0.15
            else:
                # turn right
                base_msg.angular.z = -0.15
            self.move_base_pub.publish(base_msg)
            rospy.sleep(0.6)
            self.odom_pos_3d.pose = self.curr_pos.pose
            odom_flip = tf2_geometry_msgs.do_transform_pose(self.odom_pos_3d, odom_to_map_trans)
            q = odom_flip.pose.orientation
            _, _, yaw = tf_conversions.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
            y_diff = abs(self.home.pose.position.y - odom_flip.pose.position.y)
            x_diff = abs(self.home.pose.position.x - odom_flip.pose.position.x)
            angle_needed = math.atan(y_diff / x_diff)
            dist = math.sqrt((odom_flip.pose.position.x - self.home.pose.position.x) ** 2 + (odom_flip.pose.position.y - self.home.pose.position.y) ** 2)
            if dist > dist_thresh:
                base_msg.linear.x = -0.1
                self.move_base_pub.publish(base_msg)
                rospy.sleep(0.3)
        
        rospy.loginfo("done homing")
            
        # open the gripper again
        apt = self.gc.finger_rad_to_aperture(0.2)
        self.move_to_pose({"gripper_aperture": apt})

    def normalize_angle(self, angle: float) -> float:
        """
        Input is in radians
        """
        two_pi_scoped = math.atan2(math.sin(angle), math.cos(angle))
        if two_pi_scoped < 0:
            two_pi_scoped += 2 * math.pi
        return two_pi_scoped


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

    def shutdown_hook(self):
        rospy.loginfo("Shutting down TeleopNode")
        if len(self.saved_poses):
            with open(self.saved_poses_file, "wb") as file:
                pickle.dump(self.saved_poses, file)
        