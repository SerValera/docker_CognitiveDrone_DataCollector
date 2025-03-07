#!/usr/bin/env python3

import os
import rospy
import subprocess
from gazebo_msgs.srv import SpawnModel, SetModelState, GetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
import tf.transformations as tf
import math

d_time = 0.15 #before rotation sleep

class SpawnerGates:
    def __init__(self, path, bg_path, pose=[-5, 5]):
        self.spawned_objects = {}
        self.spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.folder_path = path

        self.bg_path = "/home/sim/ardupilot_docker/droneVLA_dataset/Backgrounds"
        self.backgrounds = ["build.sdf", "city.sdf"]
        self.initial_poses = {}
        self.spawn_all_models_from_folder(self.bg_path)

        for k in range(3):
            self.spawn_gates_in_gazebo(k, init_x=pose[0]-5*k, init_y=pose[1], init_z=-3)

        self.spawned_scene_models = []

    def spawn_all_models_from_folder(self, folder_path):
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        spawn_model_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

        files = [f for f in os.listdir(folder_path) if f.endswith('.sdf')]

        x = 0
        for sdf_file in files:
            model_name = os.path.splitext(sdf_file)[0]
            sdf_file_path = os.path.join(folder_path, sdf_file)

            with open(sdf_file_path, 'r') as f:
                sdf_content = f.read()

            # Set initial pose (could be randomized or fixed, here just an example)
            init_pose = Pose()
            init_pose.position.x = x
            init_pose.position.y = 0.0
            init_pose.position.z = -10  # Slightly above ground to avoid collision
            quaternion = tf.quaternion_from_euler(0, 0, math.pi)
            init_pose.orientation.x = quaternion[0]
            init_pose.orientation.y = quaternion[1]
            init_pose.orientation.z = quaternion[2]
            init_pose.orientation.w = quaternion[3]
    
            x += 2
            try:
                spawn_model_srv(model_name, sdf_content, "", init_pose, "world")
                rospy.loginfo(f"Spawned model: {model_name}")
                self.initial_poses[model_name] = init_pose  # Store initial pose
            except rospy.ServiceException as e:
                rospy.logerr(f"Failed to spawn model {model_name}: {e}")


    def move_model_to_pose(self, model_name, x, y, z, roll=0, pitch=0, yaw=3.14):
        model_name = str(model_name)
        rospy.wait_for_service('/gazebo/set_model_state')
        set_model_state_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        state = ModelState()
        state.model_name = model_name
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = z

        # For simplicity, ignoring orientation for now (can add quaternion conversion from roll, pitch, yaw if needed)
        quaternion = tf.quaternion_from_euler(roll, pitch, yaw)
        state.pose.orientation.x = quaternion[0]
        state.pose.orientation.y = quaternion[1]
        state.pose.orientation.z = quaternion[2]
        state.pose.orientation.w = quaternion[3]

        try:
            set_model_state_srv(state)
            rospy.sleep(0.5)
            rospy.loginfo(f"Moved {model_name} to new pose")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to move model {model_name}: {e}")

    def move_model_to_initial_pose(self, model_name):
        if model_name not in self.initial_poses:
            rospy.logwarn(f"No initial pose recorded for {model_name}")
            return

        init_pose = self.initial_poses[model_name]
        self.move_model_to_pose(model_name, 
                        init_pose.position.x, 
                        init_pose.position.y, 
                        init_pose.position.z)

    def spawn_gates_in_gazebo(self, k, init_x=0, init_y=0, init_z=0, grid_size=1):       
        gate_files = [f for f in os.listdir(self.folder_path) if f.startswith("gate")]
        gate_files.sort()
        
        x, y = init_x, init_y
        for i, gate_file in enumerate(gate_files):
            model_name_gazebo = str(k) + "_" + gate_file
            gate_file = gate_file + "/model.sdf"
            model_name = gate_file.replace('.sdf', '')
            model_path = os.path.join(self.folder_path, gate_file)
            
            with open(model_path, 'r') as f:
                model_xml = f.read()
            
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = init_z
            
            try:
                self.spawn_model(model_name_gazebo, model_xml, "", pose, "world")
                rospy.loginfo(f"Spawned {model_name_gazebo} at ({x}, {y}, {init_z})")
                self.spawned_objects[model_name_gazebo] = (x, y, init_z)
            except rospy.ServiceException as e:
                rospy.logerr(f"Failed to spawn {model_name_gazebo}: {e}")
            
            x += grid_size
            if (i + 1) % 3 == 0:
                x = init_x
                y += grid_size
        
        return self.spawned_objects

    def move_gate_by_attributes(self, id, size, color, shape, x, y, z, roll, pitch, yaw):
        model_name = None
        for name in self.spawned_objects.keys():
            if str(id) in name and size in name and color in name and shape in name:
                model_name = name
                break
    

        if model_name:
            rospy.wait_for_service('/gazebo/set_model_state')
            set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            
            state = ModelState()
            state.model_name = model_name
            state.pose.position.x = x
            state.pose.position.y = y
            state.pose.position.z = z
            
            try:
                set_model_state(state)
                rospy.loginfo(f"Moved {model_name} to ({x}, {y}, {z})")
                rospy.sleep(d_time)
                self.rotate_model(model_name, roll, pitch, yaw)

                self.spawned_scene_models.append(model_name)
            except rospy.ServiceException as e:
                rospy.logerr(f"Failed to move {model_name}: {e}")


    def rotate_model(self, model_name, roll, pitch, yaw):
        rospy.wait_for_service('/gazebo/get_model_state')
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            current_state = get_state(model_name, "world")

            if current_state.success:
                quaternion = tf.quaternion_from_euler(roll, pitch, yaw)
                state = ModelState()
                state.model_name = model_name
                state.pose.position = current_state.pose.position  # Keep original position
                state.pose.orientation.x = quaternion[0]
                state.pose.orientation.y = quaternion[1]
                state.pose.orientation.z = quaternion[2]
                state.pose.orientation.w = quaternion[3]
                resp = set_state(state)
                return resp
            else:
                print(f"Failed to get state for model {model_name}")

        except rospy.ServiceException as e:
            print("Service call failed:", e)

    def move_gate_back(self):
        print("Move back")
        print(self.spawned_scene_models)

        rospy.sleep(d_time)

        for model_name in self.spawned_scene_models:

            x, y, z = self.spawned_objects[model_name]
            
            rospy.wait_for_service('/gazebo/set_model_state')
            set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            
            state = ModelState()
            state.model_name = model_name
            state.pose.position.x = x
            state.pose.position.y = y
            state.pose.position.z = z
            
            try:
                set_model_state(state)
                rospy.loginfo(f"Moved back{model_name} to ({x}, {y}, {z})")
            except rospy.ServiceException as e:
                rospy.logerr(f"Failed to move {model_name}: {e}")


        self.spawned_scene_models = []
