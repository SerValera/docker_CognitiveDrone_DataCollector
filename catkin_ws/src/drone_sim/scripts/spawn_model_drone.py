#!/usr/bin/env python3

import rospy
import csv
import os
import json
import re
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose

from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
import tf.transformations as tf

from camera_move import SpawnerCamera
from gates_move import SpawnerGates


# TO FILL BY USER
DATA_SET_TO_RECORD_PATH = rospy.get_param('/DATA_SET_TO_RECORD_PATH')
CATEGORY = rospy.get_param('/CATEGORY')
TYPE = rospy.get_param('/TYPE')
IS_MATH = rospy.get_param('/IS_MATH')
# ---------------

MODELS_PATH = "/home/sim/ardupilot_docker/catkin_ws/src/drone_sim/models"
CSV_FILE = "/home/sim/ardupilot_docker/catkin_ws/src/drone_sim/objects_tasks/scene_setup.csv"
BACKGROUND_PATH = "/home/sim/ardupilot_docker/droneVLA_dataset/Backgrounds"

class SpawnerModels:
    def __init__(self):
        self.spawned_models = set()

        self.init_pose = [-1, 0, 0.75]
        self.goal_points = []
        self.json_data = None

        self.scene_setup_csv = None
        self.setup_gates = []
        self.setup_labels = []

        self.read_setup_scene()

        self.count = 1

        self.spawn_ones = True
        self.spawn_by_id = 1

    def read_setup_scene(self):
        self.setup_gates = []
        with open(CSV_FILE, "r") as file:
            reader = csv.DictReader(file)
            self.scene_setup_csv = reader
            for row in reader:
                if row["model_name"] == "gate":
                    model_name, id, x, y, z, roll, pitch, yaw = row["model_name"], int(row["id"]), float(row["x"]), float(row["y"]), float(row["z"]), float(row["roll"]), float(row["pitch"]), float(row["yaw"])
                    self.setup_gates.append([id, x, y, z, roll, pitch, yaw])
                if row["model_name"] == "label":
                    model_name, id, x, y, z, roll, pitch, yaw = row["model_name"], int(row["id"]), float(row["x"]), float(row["y"]), float(row["z"]), float(row["roll"]), float(row["pitch"]), float(row["yaw"])
                    self.setup_labels.append([id, x, y, z, roll, pitch, yaw])

    def replace_uri_path(self, file_path, path_to_model, dae_files, i) -> None:
        """
        Читает XML из файла, заменяет путь внутри тегов <uri> на новый путь и обновляет файл.
        
        :param file_path: Путь к XML файлу.
        :param new_path: Новый путь, который нужно вставить в <uri>.
        """
        with open(path_to_model, "r", encoding="utf-8") as file:
            xml_text = file.read()
        
        updated_xml = re.sub(r'(<uri>).*?(</uri>)', rf'\1{file_path + dae_files}\2', xml_text)
        
        with open(path_to_model, "w", encoding="utf-8") as file:
            file.write(updated_xml)


    def update_sdf_file(self, file_path, dae_files, path_to_models):
        for i in range(len(path_to_models)):
            self.replace_uri_path(file_path, path_to_models[i], dae_files[i], i+1)

    def update_sdf_math_file(self, file_path, dae_files, path_to_models):
        file_names_math = [dae_files['d1'], dae_files['operator'], dae_files['d2']]
        for i in range(len(path_to_models)):
            self.replace_uri_path(file_path, path_to_models[i], str(file_names_math[i])+".dae", i+1)

    def delete_existing_models(self):
        rospy.wait_for_service("/gazebo/delete_model")
        delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        
        for model in list(self.spawned_models):
            try:
                delete_model(model)
                rospy.loginfo(f"Deleted {model}")
                self.spawned_models.remove(model)
            except rospy.ServiceException as e:
                rospy.logwarn(f"Failed to delete {model}: {e}")

    def spawn_label(self, model_name, x, y, z):
        model_sdf_file = os.path.join(MODELS_PATH, "label", "model.sdf")
        if not os.path.exists(model_sdf_file):
            rospy.logerr(f"Model SDF not found: {model_sdf_file}")
            return
        with open(model_sdf_file, "r") as f:
            model_xml = f.read()
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z + 2
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        name = model_name+'_label'
        try:
            spawn_service = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
            spawn_service(name, model_xml, "", pose, "world")
            rospy.loginfo(f"Spawned {name} at ({x}, {y}, {z})")
            self.pawned_models.add(name)
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to spawn {name}: {e}")

    def read_json_file(self, cat, type):

        file_path = DATA_SET_TO_RECORD_PATH + "/" + cat + "/" + type + ".json"

        print("file_path", file_path)

        def extract_all_data(data):
            """Extracts and stores all data for each prompt."""
            extracted_data = []
            for item in data:
                gates_info = [
                    {"gate": gate_key, **gate_value}
                    for gate_key, gate_value in item.get("gates", {}).items()
                ]
                extracted_data.append({
                    "prompt": item["prompt"],
                    # "task": item["task"],
                    "prompt_simpler": item["prompt_simpler"],
                    "options": item["options"],
                    "correct": item["correct"],
                    "gates": gates_info,
                    "background": item["background"]
                })
            return extracted_data
        try:
            with open(file_path, 'r', encoding='utf-8') as file:
                data = json.load(file)
                self.json_data = extract_all_data(data)
        except (FileNotFoundError, json.JSONDecodeError) as e:
            print(f"Error reading JSON file: {e}")
            return None
        
    def rotate_model(self, id, model_name, roll, pitch, yaw):
        gazebo_model_name = model_name + "_" + str(id)
        rospy.wait_for_service('/gazebo/get_model_state')
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            current_state = get_state(gazebo_model_name, "world")

            if current_state.success:
                quaternion = tf.quaternion_from_euler(roll, pitch, yaw)
                state = ModelState()
                state.model_name = gazebo_model_name
                state.pose.position = current_state.pose.position  # Keep original position
                state.pose.orientation.x = quaternion[0]
                state.pose.orientation.y = quaternion[1]
                state.pose.orientation.z = quaternion[2]
                state.pose.orientation.w = quaternion[3]
                resp = set_state(state)
                return resp
            else:
                print(f"Failed to get state for model {gazebo_model_name}")

        except rospy.ServiceException as e:
            print("Service call failed:", e)

    def spawn_model(self, id, model_name, x, y, z):
        model_sdf_file = os.path.join(MODELS_PATH, model_name, "model.sdf")
        
        if not os.path.exists(model_sdf_file):
            rospy.logerr(f"Model SDF not found: {model_sdf_file}")
            return

        with open(model_sdf_file, "r") as f:
            model_xml = f.read()
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        gazebo_model_name = model_name
        try:
            spawn_service = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
            spawn_service(gazebo_model_name, model_xml, "", pose, "world")
            rospy.loginfo(f"Spawned {model_name} with id {id} at ({x}, {y}, {z})")
            self.spawned_models.add(gazebo_model_name)

        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to spawn {gazebo_model_name}: {e}")

    def recording_json(self, CGates, cat, type):
        # Read json with dataset discription, for particula Category and Type
        self.read_json_file(cat, type)

        for setup in self.json_data:

            if self.spawn_by_id == self.count:

                if self.spawn_ones:
                    print("Total: " + str(self.count) + " of " + str(len(self.json_data)))
                    objects_poses = []

                    # Uptade gates
                    print("prompt", setup["prompt"])
                    for gate in setup["gates"]:
                        id = int(gate["gate"]) - 1

                        if id + 1 == int(setup["correct"]):
                            objects_poses.append([self.setup_gates[id][1], self.setup_gates[id][2], self.setup_gates[id][3]])
                            
                        CGates.move_gate_by_attributes(id, gate["size"], gate["color"], gate["shape"], self.setup_gates[id][1], self.setup_gates[id][2], self.setup_gates[id][3], self.setup_gates[id][4], self.setup_gates[id][5], self.setup_gates[id][6])
                        rospy.sleep(0.5)

                    # Update SDF labels for new task
                    path_to_models = [
                        "/home/sim/ardupilot_docker/catkin_ws/src/drone_sim/models/label_1/model.sdf",
                        "/home/sim/ardupilot_docker/catkin_ws/src/drone_sim/models/label_2/model.sdf",
                        "/home/sim/ardupilot_docker/catkin_ws/src/drone_sim/models/label_3/model.sdf",
                    ]
                    path_to_files = DATA_SET_TO_RECORD_PATH + "/" + cat + "/" + type + "/"
                    self.update_sdf_file(path_to_files, setup["options"], path_to_models)

                    # Spawn models in gazebo, remove previous
                    rospy.sleep(0.5)
                    self.load_lables()
                    rospy.sleep(0.5)

                    if IS_MATH:
                        path_to_math_models = [
                            "/home/sim/ardupilot_docker/catkin_ws/src/drone_sim/models/math_1/model.sdf",
                            "/home/sim/ardupilot_docker/catkin_ws/src/drone_sim/models/math_2/model.sdf",
                            "/home/sim/ardupilot_docker/catkin_ws/src/drone_sim/models/math_3/model.sdf",
                        ]
                        path_to_files = "/home/sim/ardupilot_docker/math/"
                        self.update_sdf_math_file(path_to_files, setup["task"], path_to_math_models)

                                # Spawn models in gazebo, remove previous
                    # rospy.sleep(0.5)
                    # self.load_math()
                    # rospy.sleep(0.1)

                    # Move camera and record
                    # CCamera.goal_poses = objects_poses
                    
                    # CCamera.move_camera(cat, type, setup)

                    # for background in CGates.initial_poses:
                    #     CGates.move_model_to_pose(str(background), 7, 0, 5)
                    #     rospy.sleep(0.05)
                    #     CCamera.move_camera(cat, type, setup)
                    #     CGates.move_model_to_pose(str(background), 10, 0, -10)
                    #     rospy.sleep(0.05)

                    # Move gate to its init poses
                    # CGates.move_gate_back()
                    self.spawn_ones = False

            self.count += 1
                

        # self.delete_existing_models()  # Clear previous scene

    def load_lables(self):
        self.delete_existing_models()  # Clear previous scene

        with open(CSV_FILE, "r") as file:
            reader = csv.DictReader(file)
            for row in reader:
                if row["model_name"] == "label":
                    model_name, id, x, y, z, roll, pitch, yaw = row["model_name"], int(row["id"]), float(row["x"]), float(row["y"]), float(row["z"]), float(row["roll"]), float(row["pitch"]), float(row["yaw"])
                    self.spawn_model(id, model_name + "_" + row["id"], x, y, z)
                    rospy.sleep(0.25)
                    self.rotate_model(id, model_name, roll, pitch, yaw)

    def load_math(self):
        with open(CSV_FILE, "r") as file:
            reader = csv.DictReader(file)
            for row in reader:
                if row["model_name"] == "math":
                    model_name, id, x, y, z, roll, pitch, yaw = row["model_name"], int(row["id"]), float(row["x"]), float(row["y"]), float(row["z"]), float(row["roll"]), float(row["pitch"]), float(row["yaw"])
                    self.spawn_model(id, model_name + "_" + row["id"], x, y, z)
                    rospy.sleep(0.25)
                    self.rotate_model(id, model_name, roll, pitch, yaw)


    def load_scene(self, setup_name, gates):
        self.delete_existing_models()  # Clear previous scene

        with open(CSV_FILE, "r") as file:
            reader = csv.DictReader(file)
            objects_poses = []
            for row in reader:
                if row["setup"] == setup_name:
                    model_name, id, x, y, z, roll, pitch, yaw = row["model_name"], int(row["id"]), float(row["x"]), float(row["y"]), float(row["z"]), float(row["roll"]), float(row["pitch"]), float(row["yaw"])
                    if model_name != "label":
                        objects_poses.append([x, y])
                    self.spawn_model(id, model_name, x, y, z)
                    rospy.sleep(0.15)
                    self.rotate_model(id, model_name, roll, pitch, yaw)

        return objects_poses

if __name__ == "__main__":
    rospy.init_node('spawn_scene', anonymous=True)

    # Prepare scene and gazebo
    spawn = SpawnerModels()
    # Ccamera = SpawnerCamera(init_point=spawn.init_pose)
    CGates = SpawnerGates(path=MODELS_PATH, bg_path=BACKGROUND_PATH, pose=[-5, 5])

    # Read task list (jsons) for data recording
    spawn.recording_json(CGates, CATEGORY, TYPE)



    