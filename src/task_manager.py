#!/usr/bin/env python3

import sys
import os
from collections import deque
import time
import argparse

import rospy
import rospkg
from std_msgs.msg import String
import actionlib

sys.path.append('../..')
from ros_chariot_master.msg import TaskActionAction, TaskActionGoal, TaskActionFeedback, TaskActionResult


class TaskManager:
    def __init__(self,agent_type="robochair",active_agents=['robotchair']):
        """
        TaskManager handles sending or executing tasks depending on the agent type.

        Input: 
            - agnet_type (str): robochair, hoyer_sling, bed, policy_server
            - activte_agents (list): ["robochair", "bed", "hoyer_sling"], do not include policy server

        Available task: 
            - kinova_grasp (hook_id:1~4, cmd:'full')
            - kinova_tie (hook_id, cmd)
            - wheelchair_move (hook_id, cmd:'full')
            - hoyer_sling_base ()
            - hoyer_sling_lift
            - hoyer_sling_bar
        """
        self.task_running = False
        self.current_task = None
        self.agent_type = agent_type
        self.task_queue = deque()
        self.active_agents = active_agents

        # start action server "/task_action" to request task execution 
        if self.agent_type == "policy_server":
            self.clients: dict[str, actionlib.SimpleActionClient] = {}

            for agent in self.active_agents:
                topic = f'/{agent}/task_action'
                print(f"[policy_server] Waiting for action server at {topic}...")
                client = actionlib.SimpleActionClient(topic, TaskActionAction)
                client.wait_for_server()
                print(f"[policy_server] Connected to {topic}")
                self.clients[agent] = client
        else: 
            # for agents, use agents' namesapce/task_action
            self.server = actionlib.SimpleActionServer(f'{self.agent_type}/task_action', TaskActionAction, self._execute_cb, False)
            self.server.start()
            print(f"[{self.agent_type}/task_manager] Action server started")

        self._load_modules()
    
        print(f"[{self.agent_type}] Task manager initialzed")

    def _load_modules(self):
        if self.agent_type == "robochair":
            rospack = rospkg.RosPack()
            pkg_path = rospack.get_path('ros_chariot_master')
            kinova_path = os.path.join(pkg_path, 'src/robochair/kinova_ctrl')
            sys.path.append(kinova_path)
            from kinova import KinovaArm

            from robochair.kinova_ctrl.kinova_motions.kinova_grasp_strap import Grasp_Strap
            from robochair.wheelchair_ctrl.wheelchair_nav_pc.wheelchair_nav import WC_MotionCommander
            from robochair.kinova_ctrl.kinova_motions.kinova_tie_strap import Tie_Strap
            
            self.robot = KinovaArm()
            self.kinova_grasp = Grasp_Strap(self.robot)
            self.wc = WC_MotionCommander()
            self.kinova_tie = Tie_Strap(self.robot)

        elif self.agent_type == "hoyer_sling":
            # TODO: implement loading for hoyer_sling components
            pass
        elif self.agent_type == "policy_server":
            from policy_server.policy_read_yaml import read_policy_yaml
            from policy_server.policy_kinova_bridge import KinovaBridge
            self.read_policy_yaml = read_policy_yaml
            self.policy_server_kinova_tie = KinovaBridge()
        print(f"[{self.agent_type}] Modules loaded")

    
    # For agent
    def _execute_cb(self, goal: TaskActionGoal):
        """
        ActionServer callback: receives a task goal, executes it, and returns result.
        """
        print(f"[{self.agent_type}] Executing task: {goal}")
        task_id = goal.task_id
        hook_id = goal.hook_id
        command = goal.command

        feedback = TaskActionFeedback()
        result = TaskActionResult()

        try:
            print(hook_id)
            executed_cmd, status = self._agent_execute_task(task_id, hook_id, command)
            
            feedback.status = f"Executing {task_id}"
            self.server.publish_feedback(feedback)

            result.success = status
            result.executed_cmd = task_id
            self.server.set_succeeded(result)

        except Exception as e:
            rospy.logerr(f"[{self.agent_type}] Task error: {e}")
            result.success = False
            result.executed_cmd = task_id
            self.server.set_aborted(result)


    def _agent_execute_task(self, task_id, hook_id, command):
        if self.agent_type == "robochair":
            # robochair task execution
            if task_id == "kinova_grasp":
                executed_cmd,status = self.kinova_grasp.run(hook_id=hook_id, cmd=command)
                return executed_cmd,status
            elif task_id == "wheelchair_move":
                executed_cmd,status = self.wc.run(hook_id=hook_id, cmd=command)
                return executed_cmd,status 
            elif task_id == "kinova_tie":
                executed_cmd,status = self.kinova_tie.run(hook_id,command)
            else:
                rospy.logwarn(f"[{self.agent_type}/task_manager] Unknown task_id: {task_id}")
                return "(UNKNOWN)", "False"
        
        elif self.agent_type == "hoyer_sling":
            return "(INOP.)", "False"
        elif self.agent_type == "bed":
            return "(INOP.)", "False"
        else:
            return "(UNKNOWN)", "False"
        

    def run(self):
        if self.agent_type == "policy_server":
            rospack = rospkg.RosPack()
            pkg_path = rospack.get_path('ros_chariot_master')
            path = os.path.join(pkg_path, 'src/policy_server/config/task_seq.yaml')
            task_seq = self.read_policy_yaml(path=path)

            for task in task_seq:
                goal = TaskActionGoal()
                goal.task_id = task['task_id']
                goal.hook_id = task.get('hook_id', 0)
                goal.command = task.get('command', "stop")

                task_agent = task.get('agent', '')

                rospy.loginfo(f"[policy_server] Sending goal: {goal}")
                self.clients[task_agent].send_goal(goal)
                # add something here for kinova_tying task on the poilcy side
                # e.g. use the self.policy_server_kinova_tie
                self.clients[task_agent].wait_for_result()

                result = self.clients[task_agent].get_result()
                print(f"[policy_server] Result: {result}")
        else:
            rospy.spin()



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run TaskManager for specified agent")
    parser.add_argument("--agent_name", type=str, default="robochair",
                        help="Agent name (default: robochair). Options: robochair, hoyer_sling, bed, policy_server")
    args = parser.parse_args()

    agent_name = args.agent_name
    rospy.init_node(f"{agent_name}_task_manager")
    manager = TaskManager(agent_name)
    manager.run()
