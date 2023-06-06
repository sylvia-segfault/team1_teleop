#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
from team1_teleop.msg import PplPos

import math
import threading    
import numpy as np
import threading
import ros_numpy as rn

import hello_helpers.hello_misc as hm
import stretch_funmap.navigate as nv


class PplLocateNode(hm.HelloNode):

    def __init__(self):
        hm.HelloNode.__init__(self)
        self.rate = 10.0
        self.joint_states = None
        self.joint_states_lock = threading.Lock()
        self.move_base = nv.MoveBase(self)
        self.letter_height_m = 0.2
        self.wrist_position = None
        self.manipulation_view = None
        
        marker = Marker()
        self.mouth_marker_type = marker.CUBE
        self.mouth_point = None

        num_pan_angles = 5

        self.sub_walker_ret = rospy.Subscriber('return_walker_cmd', Float64, self.return_walker_callback)

        # looking out along the arm
        middle_pan_angle = -math.pi/2.0

        look_around_range = math.pi
        min_pan_angle = middle_pan_angle - (look_around_range / 2.0)
        # max_pan_angle = middle_pan_angle + (look_around_range / 2.0)
        # pan_angle = min_pan_angle
        pan_increment = look_around_range / float(num_pan_angles - 1.0)
        self.pan_angles = [min_pan_angle + (i * pan_increment)
                           for i in range(num_pan_angles)]
        self.pan_angles = self.pan_angles + self.pan_angles[1:-1][::-1]
        self.prev_pan_index = 0
        
        self.move_lock = threading.Lock()

        with self.move_lock: 
            self.handover_goal_ready = False

    def joint_states_callback(self, joint_states):
        with self.joint_states_lock: 
            self.joint_states = joint_states
        wrist_position, _, _ = hm.get_wrist_state(joint_states)
        self.wrist_position = wrist_position

    def look_around_callback(self):
        # print("looking around")
        # Cycle the head back and forth looking for a person to whom
        # to handout the object.
        with self.move_lock:  
            pan_index = (self.prev_pan_index + 1) % len(self.pan_angles)
            pan_angle = self.pan_angles[pan_index]
            # print(pan_angle)
            pose = {'joint_head_pan': pan_angle}
            self.move_to_pose(pose)
            self.prev_pan_index = pan_index
        
    def mouth_position_callback(self, marker_array):
        with self.move_lock: 

            for marker in marker_array.markers:
                if marker.type == self.mouth_marker_type:
                    mouth_position = marker.pose.position
                    self.mouth_point = PointStamped()
                    self.mouth_point.point = mouth_position
                    header = self.mouth_point.header
                    header.stamp = marker.header.stamp
                    header.frame_id = marker.header.frame_id
                    header.seq = marker.header.seq
                    print('******* new mouth point received *******')

                    lookup_time = rospy.Time(0) # return most recent transform
                    timeout_ros = rospy.Duration(0.1)

                    old_frame_id = self.mouth_point.header.frame_id
                    # new_frame_id = 'base_link'
                    new_frame_id = 'map'
                    stamped_transform = self.tf2_buffer.lookup_transform(new_frame_id, old_frame_id, lookup_time, timeout_ros)
                    points_in_old_frame_to_new_frame_mat = rn.numpify(stamped_transform.transform)
                    camera_to_base_mat = points_in_old_frame_to_new_frame_mat

                    grasp_center_frame_id = 'link_grasp_center'
                    stamped_transform = self.tf2_buffer.lookup_transform(new_frame_id, grasp_center_frame_id, lookup_time, timeout_ros)
                    grasp_center_to_base_mat = rn.numpify(stamped_transform.transform)

                    mouth_camera_xyz = np.array([0.0, 0.0, 0.0, 1.0])
                    mouth_camera_xyz[:3] = rn.numpify(self.mouth_point.point)[:3]

                    mouth_xyz = np.matmul(camera_to_base_mat, mouth_camera_xyz)[:3]
                    fingers_xyz = grasp_center_to_base_mat[:,3][:3]

                    # attempt to handoff the object at a location in front
                    # the mouth with respect to the world frame (i.e.,
                    # gravity)
                    target_offset_xyz = np.array([-0.0, 0.0, 0.0])

                    target_xyz = mouth_xyz + target_offset_xyz
                    print('target_xyz =', target_xyz)

                    fingers_error = target_xyz - fingers_xyz

                    delta_forward_m = fingers_error[0] 

                    self.mouth_position_publisher.publish(
                                                        target_xyz[0],
                                                        target_xyz[1],
                                                        delta_forward_m)
                    

    
    def main(self):
        hm.HelloNode.main(self, 'find_mouth', 'find_mouth', wait_for_first_pointcloud=False)

        self.joint_states_subscriber = rospy.Subscriber('/stretch/joint_states', JointState, self.joint_states_callback)
        
        self.mouth_position_subscriber = rospy.Subscriber('/nearest_mouth/marker_array', MarkerArray, self.mouth_position_callback)

        self.mouth_position_publisher = rospy.Publisher('/ppl_locate_result', PplPos, queue_size=1)

    
    def return_walker_callback(self, data):
        print("received command to return walker")
        
        # This rate determines how quickly the head pans back and forth.
        rate = rospy.Rate(0.5)
        look_around = True
        while not rospy.is_shutdown():
            if look_around: 
                self.look_around_callback()
            rate.sleep()
        
    def shutdown_hook(self):
        rospy.loginfo("Shutting down Ppl Locate node")
