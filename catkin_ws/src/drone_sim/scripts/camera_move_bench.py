#!/usr/bin/env python3
import rospy
import numpy as np
import random
from gazebo_msgs.srv import SetModelState, GetModelState
from gazebo_msgs.msg import ModelState
from tf.transformations import quaternion_from_euler
import cv2
from spline_traj import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import csv
from datetime import datetime
import os
import json
import tf

    
LOG_FILE = "/home/sim/ardupilot_docker/goal_reaching_log.csv"

class BenchCamera:
    def __init__(self, init_point = [0,0,0], goal_pooints=[]):
        # rospy.init_node('move_model_to_goal')
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        # Начальная и конечная позиция
        self.init_pose = init_point
        self.goal_poses = goal_pooints
        
        self.init_pose_rand = [0,0,0]
        self.init_noised_distance = 1.0
        self.set_init_pose()

        self.fixed_z = 0.75

        self.frame_id = 0
        self.bridge = CvBridge()
        self.color_image = None
        self.writer_file = None
        rospy.Subscriber("/camera_1/image_raw_1", Image, self.image_callback) # main for data recording

    def quaternion_to_yaw_pitch_roll(self, q0, q1, q2, q3):
        # Roll (rotation around x-axis)
        sinr_cosp = 2 * (q0 * q1 + q2 * q3)
        cosr_cosp = 1 - 2 * (q1 * q1 + q2 * q2)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (rotation around y-axis)
        sinp = 2 * (q0 * q2 - q3 * q1)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # Yaw (rotation around z-axis)
        siny_cosp = 2 * (q0 * q3 + q1 * q2)
        cosy_cosp = 1 - 2 * (q2 * q2 + q3 * q3)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return yaw, pitch, roll

    # Function to rotate the velocity vector in the XY plane
    def rotate_velocity_xy(self, vx, vy, delta_yaw):
        cos_yaw = np.cos(delta_yaw)
        sin_yaw = np.sin(delta_yaw)
        vx_rot = vx * cos_yaw - vy * sin_yaw
        vy_rot = vx * sin_yaw + vy * cos_yaw
        return vx_rot, vy_rot

    def image_callback(self, msg):
        try:
            cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        self.color_image = np.asanyarray(cv2_img)

    def save_image(self, filename: str):
        if not os.path.exists(self.folder+"/images"):
            os.makedirs(self.folder+"/images")
        filepath = os.path.join(self.folder+"/images", filename)
        cv2.imwrite(filepath, self.color_image)

    def set_init_pose(self):
        state_msg = ModelState()
        state_msg.model_name = 'iris_demo'  # Имя модели
        state_msg.pose.position.x = self.init_pose[0]
        state_msg.pose.position.y = self.init_pose[1]
        state_msg.pose.position.z = self.init_pose[2]
        self.set_state(state_msg)

    def get_yaw_from_quaternion(self, quat):
        (_, _, yaw) = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        return yaw


    def initialize_csv(self):
        """Create the CSV file if it doesn't exist and write the header."""
        if not os.path.exists(LOG_FILE):
            with open(LOG_FILE, mode='w') as file:
                writer = csv.writer(file)
                writer.writerow(["result", "CATEGORY", "TYPE"])

    def log_result(self, result, CATEGORY, TYPE):
        """
        Log the result (success/fail), final distance to goal, and final coordinates into CSV file.
        """
        with open(LOG_FILE, mode='a') as file:
            writer = csv.writer(file)
            writer.writerow([result, CATEGORY, TYPE])

    def get_distance(self, x1, y1, z1, x2, y2, z2):
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2 + (z1 - z2)**2)

    def move_camera(self, CATEGORY, TYPE, setup): 
        for goal_pose in self.goal_poses:

            model_name = 'iris_demo'  # replace with your actual model name in Gazebo

            # Set goal position
            goal_x = goal_pose[0]
            goal_y = goal_pose[1]
            goal_z = 0.75

            # Action vector in LOCAL frame (drone body frame)
            dx = 0.1   # forward in body frame
            dy = 0.0   # sideways (0 means no strafing)
            dz = 0.0   # vertical movement
            angle = 0.0  # yaw change (radians)

            rospy.wait_for_service('/gazebo/get_model_state')
            rospy.wait_for_service('/gazebo/set_model_state')

            get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

            rate = rospy.Rate(20)  # 2 Hz loop rate

            while not rospy.is_shutdown():
                # Get current state
                resp = get_model_state(model_name, 'world')

                current_x = resp.pose.position.x
                current_y = resp.pose.position.y
                current_z = resp.pose.position.z
                current_yaw = self.get_yaw_from_quaternion(resp.pose.orientation)

                # Check distance to goal
                distance_to_goal = self.get_distance(current_x, current_y, current_z, goal_x, goal_y, goal_z)

                A = np.array([current_x, current_y, current_z])
                B = np.array([goal_x, goal_y, goal_z])

                v = B - A
                norm_v = v / np.linalg.norm(v)
                dx = norm_v[0] / 10   # forward in body frame
                dy = norm_v[1] / 10   # sideways (0 means no strafing)
                dz = norm_v[2] / 10   # vertical movement

                if distance_to_goal < 1.0:
                    rospy.loginfo("Goal reached! Distance = %.2f meters", distance_to_goal)
                    self.log_result("SUCCESS", CATEGORY, TYPE)
                    break

                if distance_to_goal > 10.0:
                    rospy.logwarn("Model too far from goal (%.2f meters). Exiting.", distance_to_goal)
                    self.log_result("FAIL", CATEGORY, TYPE)
                    break

                # ---- Transform local movement (dx, dy) into global movement ----
                new_x = current_x + dx * math.cos(current_yaw) - dy * math.sin(current_yaw)
                new_y = current_y + dx * math.sin(current_yaw) + dy * math.cos(current_yaw)
                new_z = current_z + dz  # no rotation needed for z (vertical is global up)

                # Update yaw (add angle action to current yaw)
                new_yaw = current_yaw + angle
                new_quat = tf.transformations.quaternion_from_euler(0, 0, new_yaw)

                # Prepare new model state
                model_state = ModelState()
                model_state.model_name = model_name
                model_state.pose.position.x = new_x
                model_state.pose.position.y = new_y
                model_state.pose.position.z = new_z
                model_state.pose.orientation.x = new_quat[0]
                model_state.pose.orientation.y = new_quat[1]
                model_state.pose.orientation.z = new_quat[2]
                model_state.pose.orientation.w = new_quat[3]

                try:
                    resp = set_model_state(model_state)
                    if not resp.success:
                        rospy.logwarn("Failed to set model state")
                except rospy.ServiceException as e:
                    rospy.logerr("Service call failed: %s", str(e))

                rospy.loginfo("Moved to (%.2f, %.2f), Distance to goal: %.2f meters", new_x, new_y, distance_to_goal)

                rate.sleep()


        self.set_init_pose()
