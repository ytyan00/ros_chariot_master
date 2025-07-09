import yaml

def wheelchair_load_pid_config(config_path='src/robochair/wheelchair_ctrl/config/wheelchair_PID.yaml'):
    """
    Loads PID controller parameters from a YAML file.

    Args:
        config_path (str): Path to the YAML config file.

    Returns:
        dict: Dictionary containing PID parameters for 'forward', 'backward', and 'angular'.
    """
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
        # print(config)
        return config
    
if __name__ == "__main__":
    wheelchair_load_pid_config()
