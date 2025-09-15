import json
import numpy as np
import matplotlib.pyplot as plt

def parse_scenario(json_file):
    with open(json_file, "r") as f:
        data = json.load(f)

    # Driving mode
    status_adf = data["ego_vehicle_systems"][0]["status_adf"]
    driving_mode = "manual" if status_adf == 0 else "automated"

    # Ego dynamics
    ego_times = np.array([step["file_time"] for step in data["ego_vehicle_dynamics"]])
    ego_velocities = np.array([step["velocity"] for step in data["ego_vehicle_dynamics"]])
    ego_positions = np.array([(step["x"], step["y"]) for step in data["ego_vehicle_dynamics"]])
    ego_accelerations = np.array([(step["ax"], step["ay"]) for step in data["ego_vehicle_dynamics"]])
    ego_abs_acceleration = np.array([np.sqrt(step["ax"]**2 + step["ay"]**2) for step in data["ego_vehicle_dynamics"]])
    ego_abs_position = np.array([np.sqrt(step["x"]**2 + step["y"]**2) for step in data["ego_vehicle_dynamics"]])
    ego_heading_angle = np.array([step["heading_angle"] for step in data["ego_vehicle_dynamics"]])
    ego_yaw_rate = np.array([step["yaw_rate"] for step in data["ego_vehicle_dynamics"]])


    # Dynamic objects
    dynamic_objects = {}
    for obj in data.get("dynamic_objects", []):
        if obj["abs_velocity"] > 0:
            obj_id = obj["internal_id"]
            if obj_id not in dynamic_objects:
                dynamic_objects[obj_id] = {
                    "time": [],
                    "velocity": [],
                    "position": [],
                    "position_abs": [],
                    "acceleration": [],
                    "acceleration_abs": [],
                    "rel_position": [],
                    "rel_velocity": [],
                    "length": [],
                    "width": [],
                }
            dynamic_objects[obj_id]["time"].append(obj["file_time"])
            dynamic_objects[obj_id]["velocity"].append(obj["abs_velocity"])
            dynamic_objects[obj_id]["position"].append((obj["x"], obj["y"]))
            dynamic_objects[obj_id]["position_abs"].append(np.sqrt(obj["x"]**2 + obj["y"]**2))
            dynamic_objects[obj_id]["acceleration"].append((obj["ax"], obj["ay"]))
            dynamic_objects[obj_id]["acceleration_abs"].append(np.sqrt(obj["ax"]**2 + obj["ay"]**2))
            dynamic_objects[obj_id]["rel_position"].append((obj["rel_position_x"], obj["rel_position_y"]))
            dynamic_objects[obj_id]["rel_velocity"].append((obj["rel_velocity_x"], obj["rel_velocity_y"]))
            dynamic_objects[obj_id]["length"].append(obj["length"])
            dynamic_objects[obj_id]["width"].append(obj["width"])

    # Convert lists to arrays
    for obj_id, obj_data in dynamic_objects.items():
        dynamic_objects[obj_id] = {
            "time": np.array(obj_data["time"]),
            "velocity": np.array(obj_data["velocity"]),
            "position": np.array(obj_data["position"]),
            "acceleration": np.array(obj_data["acceleration"]),
        },
        obj_velocities = np.array(obj_data["velocity"])

    return {
        "driving_mode": driving_mode,
        "mean ego velocity": ego_velocities.mean(),
        "ego": {
            "time": ego_times,
            "velocity": ego_velocities,
            "position": ego_positions,
            "position_abs": ego_abs_position,
            "acceleration": ego_accelerations,
            "acceleration_abs": ego_abs_acceleration,
            "heading_angle": ego_heading_angle,
            "yaw_rate": ego_yaw_rate,
        },
        "dynamic_objects": dynamic_objects,
        "mean object velocity": obj_velocities.mean() if dynamic_objects else 0,
    }


