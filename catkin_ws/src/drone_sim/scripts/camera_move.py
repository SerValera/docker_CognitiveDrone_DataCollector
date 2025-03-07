#!/usr/bin/env python3
import rospy
import numpy as np
import random
from gazebo_msgs.srv import SetModelState
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


class SpawnerCamera:
    def __init__(self, init_point = [0,0,0], goal_pooints=[]):

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
        rospy.Subscriber("/camera_1/image_raw_1", Image, self.image_callback)

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

    def get_normalized_vector(self, point1, point2):
        vector = np.array(point2) - np.array(point1)
        norm = np.linalg.norm(vector)
        if norm == 0:
            raise ValueError("The points are the same, no direction vector.")
        normalized_vector = vector / norm
        return normalized_vector


    def rotate_velocity_xy(self, vx, vy, delta_yaw):
        cos_yaw = np.cos(delta_yaw)
        sin_yaw = np.sin(delta_yaw)
        vx_rot = vx * cos_yaw - vy * sin_yaw
        vy_rot = vx * sin_yaw + vy * cos_yaw
        return vx_rot, vy_rot

    def create_folder(self):
        " Create folder to store data set "
        if not os.path.exists(self.folder):
            os.makedirs(self.folder)
            print("Directory created successfully!")
        else:
            print("Directory already exists!") 

        if not os.path.exists(self.folder+"/images"):
            os.makedirs(self.folder+"/images")
        else:
            print("Directory already exists!") 

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
        state_msg.model_name = 'iris_demo'
        state_msg.pose.position.x = self.init_pose[0]
        state_msg.pose.position.y = self.init_pose[1]
        state_msg.pose.position.z = self.init_pose[2]
        self.set_state(state_msg)

    def init_folders_to_store_data(self, CATEGORY, TYPE, setup):
        self.folder = "/home/sim/ardupilot_docker/recorded_dataset/" + CATEGORY + "/" + TYPE + "/" + str(datetime.now()) + "/"
        self.create_folder()
        with open(self.folder + "data.csv", 'w', newline='') as self.writer_file:
            self.writer_csv = csv.writer(self.writer_file)
            self.writer_csv.writerow(['frame_id', 'dx', 'dy', 'dz','angle', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])

        with open(self.folder + "setup.json", 'w') as json_file:
            json.dump(setup, json_file, indent=4)

    def move_camera(self, CATEGORY, TYPE, setup): 
        for goal_pose in self.goal_poses:

            self.init_folders_to_store_data(CATEGORY, TYPE, setup)
            self.frame_id = 0

            self.init_pose_rand[0] = self.init_pose[0] + random.uniform(-self.init_noised_distance, self.init_noised_distance)
            self.init_pose_rand[1] = self.init_pose[1] + random.uniform(-self.init_noised_distance, self.init_noised_distance)
            self.init_pose_rand[2] = self.init_pose[2] + random.uniform(-self.init_noised_distance, self.init_noised_distance)
            
            if self.init_pose_rand[2] <= 0.2:
                self.init_pose_rand[2] = 0.2

            x_vals,y_vals = main_2d(self.init_pose_rand, goal_pose)
            z_vals = np.linspace(self.init_pose_rand[2], self.fixed_z, len(x_vals))

            position_noise_range = 0.0
            orientation_noise_range = 0.0

            step = 3
            setp_loop = step + 2
            for i in range(len(x_vals)-setp_loop):
                x = x_vals[i] + random.uniform(-position_noise_range, position_noise_range)
                y = y_vals[i] + random.uniform(-position_noise_range, position_noise_range)
                z = z_vals[i] + random.uniform(-position_noise_range, position_noise_range)

                dx = x_vals[i] - x_vals[i+step]
                dy = y_vals[i] - y_vals[i+step]
                dz = z_vals[i] - z_vals[i+step]
                angle = np.arctan2(-dy, -dx)
                    
                # Ориентация с шумом
                noise_roll = random.uniform(-orientation_noise_range, orientation_noise_range)
                noise_pitch = random.uniform(-orientation_noise_range, orientation_noise_range)
                noise_yaw = random.uniform(-orientation_noise_range, orientation_noise_range)

                noisy_angle = angle + noise_yaw
                
                quat = quaternion_from_euler(noise_roll, noise_pitch, noisy_angle)
                
                state_msg = ModelState()
                state_msg.model_name = 'iris_demo'
                state_msg.pose.position.x = x
                state_msg.pose.position.y = y
                state_msg.pose.position.z = z
                state_msg.pose.orientation.x = quat[0]
                state_msg.pose.orientation.y = quat[1]
                state_msg.pose.orientation.z = quat[2]
                state_msg.pose.orientation.w = quat[3]

                self.set_state(state_msg)
                
                """ The vector is directed towards the gate """
                    # Current orientation
                dx_next = x_vals[i+step] - goal_pose[0]
                dy_next = y_vals[i+step] - goal_pose[1]
                angle_next = np.arctan2(-dy_next, -dx_next)  # угол для ориентации
                delta_yaw = angle_next - angle

                if abs(delta_yaw) > np.pi/2:
                    continue  # Skip this row if the condition is met

                # Rotation matrix for yaw angle (2D rotation, assuming yaw is in radians)
                cos_yaw = np.cos(angle)
                sin_yaw = np.sin(angle)

                # Rotate the velocity vector into the local frame
                local_velocity_x = cos_yaw * dx + sin_yaw * dy
                local_velocity_y = -sin_yaw * dx + cos_yaw * dy
                local_velocity_z = dz  # Assuming no rotation for the z-axis
                """ --- """

                # Save trajectories
                if self.frame_id > 0: #FIXME: if frame id == 0 image came from last of the previuse flight.
                    with open(self.folder + 'data.csv', 'a', newline='') as file:
                        self.writer_file = csv.writer(file)
                        self.writer_file.writerow([self.frame_id,local_velocity_x,local_velocity_y,local_velocity_z,delta_yaw,x,y,z,quat[0],quat[1],quat[2],quat[3]])

                    # Save images
                    self.save_image('image_'+str(self.frame_id)+'.jpg')

                rospy.sleep(0.0075)
                self.frame_id += 1

            rospy.loginfo("Model reached goal position.")

        self.set_init_pose()
