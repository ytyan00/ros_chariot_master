import yaml

def get_grasp_initial_poses(yaml_path,hook_num:int):
    """
    Load a dict of named 7-DOF poses from a YAML file.\n
    Inputs: yaml_path default to grasp_init_pos.yaml\n
            hook_num: int, the strap working on\n
    Returns: { pose_name: [j1,...,j7], ... }\n
    """
    hook_key = "hook"+str(hook_num)
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    poses = data.get('initial_poses', {})
    # sanity check: each pose must be a list of length 7
    for name, joint_list in poses.items():
        if not isinstance(joint_list, list) or len(joint_list) != 7:
            raise ValueError(f"Pose '{name}' is not a 7-element list")
    return poses[hook_key]



if __name__ == '__main__':
    poses = get_grasp_initial_poses('/home/yunting/catkin_ws/src/ros_chariot_master/src/kinova_control/grasp_init_pos.yaml',1)
    # Example: retrieve the "ready" pose
    print(poses)