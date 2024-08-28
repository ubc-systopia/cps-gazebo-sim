import json
import os
from ament_index_python import get_package_share_directory


def read_config(file_path):
    # Read the json file with robot arms
    with open(file_path, "r") as openfile:
        arms = json.load(openfile)["robot_arms"]
    if arms is None:
        raise Exception("No robot arms configuration found") 
    return arms

def get_brands(robot_arms_config):
    return [arm["brand"] for arm in robot_arms_config]

def get_arm_instances_by_brand(robot_arms_config: list[dict], brand: str) -> list[dict]:
    for arm in robot_arms_config:
        if arm["brand"] == brand:
            return arm["instances"]

def main(args=None):
    file_path = os.path.join(get_package_share_directory('multi_arm_lab_sim_bringup'), "config", "arms.json")
    arms_config = read_config(file_path=file_path)
    print(get_brands(arms_config))

    universal_robots = get_arm_instances_by_brand(arms_config, "Universal Robots")
    print(universal_robots)

    interbotix_xsarms = get_arm_instances_by_brand(arms_config, "Interbotix")
    print(interbotix_xsarms)

if __name__ == '__main__':
    main()
