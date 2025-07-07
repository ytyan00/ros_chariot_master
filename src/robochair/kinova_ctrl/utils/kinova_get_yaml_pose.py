import yaml

def get_yaml_poses(yaml_path,hook_num:int):
    """
    Load a dict of named 7-DOF poses from a YAML file.\n
    Inputs: yaml_path default to grasp_init_pos.yaml\n
            hook_num: int, the strap working on\n
    Returns: { pose_name: [j1,...,j7], ... }\n
    """
    if hook_num < 0:
        hook_key = "final_hook"+str(abs(hook_num))
    else:
        hook_key = "hook"+str(hook_num)
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    poses = data.get('initial_poses', {})
    return poses[hook_key]



if __name__ == '__main__':
    # poses = get_yaml_poses('/home/chariot/catkin_ws/src/robochair/src/kinova_ctrl/config/grasp_init_pos.yaml',1)
    poses = get_yaml_poses('/home/chariot/catkin_ws/src/robochair/src/kinova_ctrl/config/retract.yaml',-1)

    # Example: retrieve the "ready" pose
    print(poses)