import os
import json

base_folder = "/home/vs/ardupilot_docker/Validation/Validation_reasoning/Validation/validation_logo"

combined_file = "validation_logo.json"

all_data = []

for folder_name in os.listdir(base_folder):
    folder_path = os.path.join(base_folder, folder_name)

    if os.path.isdir(folder_path):
        json_file_path = os.path.join(folder_path, "setup.json")

        # Check if prompt.json exists
        if os.path.exists(json_file_path):
            with open(json_file_path, "r", encoding="utf-8") as file:
                try:
                    data = json.load(file)
                    all_data.append(data)
                except json.JSONDecodeError as e:
                    print(f"Error reading {json_file_path}: {e}")

with open(combined_file, "w", encoding="utf-8") as outfile:
    json.dump(all_data, outfile, indent=4, ensure_ascii=False)

print(f"Combined {len(all_data)} JSON files into '{combined_file}'")
