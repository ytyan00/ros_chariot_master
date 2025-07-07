import yaml

import yaml
import os

def read_policy_yaml(path=None):
    """
    Reads a YAML file and returns a list of task dictionaries.
    Default path is './policy.yaml' in the current working directory.
    """
    if path is None:
        path = os.path.join(os.path.dirname(__file__), 'policy.yaml')

    with open(path, 'r') as f:
        task_list = yaml.safe_load(f)
    
    # Sanitize and fill defaults
    for task in task_list:
        if 'hook_id' not in task:
            task['hook_id'] = 0
        if 'command' not in task:
            task['command'] = ''
        if 'agent' not in task:
            task['agent'] = ''
    return task_list 


if __name__ == '__main__':
    seq = read_policy_yaml('/home/chariot/catkin_ws/src/ros_chariot_master/src/policy_server/config/task_seq.yaml')

    print(seq)