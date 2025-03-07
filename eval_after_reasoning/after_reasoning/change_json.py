import os
import json

# Folder with your JSON files
folder_path = '/home/vs/ardupilot_docker/eval_after_reasoning/after_reasoning'  # Change this to your folder path

def convert_gates_list_to_dict(gates_list):
    """
    Convert list of gates with 'gate' keys into a dictionary where the key is the gate number.
    Keeps all other fields untouched (size, shape, color).
    """
    gates_dict = {}
    for gate in gates_list:
        gate_number = gate.pop('gate')
        gates_dict[gate_number] = gate
    return gates_dict

# Process each JSON file in the folder
for filename in os.listdir(folder_path):
    if filename.endswith('.json'):
        file_path = os.path.join(folder_path, filename)

        # Load the JSON file
        with open(file_path, 'r', encoding='utf-8') as file:
            data = json.load(file)

        # Process each item in the list (assuming the JSON is a list of objects)
        for item in data:
            if 'gates' in item and isinstance(item['gates'], list):
                item['gates'] = convert_gates_list_to_dict(item['gates'])

        # Save to new file (preserve original by creating _updated version)
        new_filename = filename.replace('.json', '_updated.json')
        new_file_path = os.path.join(folder_path, new_filename)

        with open(new_file_path, 'w', encoding='utf-8') as file:
            json.dump(data, file, indent=4, ensure_ascii=False)

        print(f'Processed and saved: {new_file_path}')
