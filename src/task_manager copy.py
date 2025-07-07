#!/usr/bin/env python3

import sys
import os
from collections import deque
import time


import rospy
import rospkg
from std_msgs.msg import String
import actionlib

sys.path.append('../..')
from ros_chariot_master.msg import TaskActionAction, TaskActionGoal, TaskActionFeedback, TaskActionResult


class TaskManager:
    def __init__(self,agent_type="robochair"):
        """
        TaskManager handles sending or executing tasks depending on the agent type.

        Input: 
            - agnet_type (str): robochair, hoyer_sling, bed, policy_server

        Available task: 
            - kinova_grasp (hook_id:1~4, cmd:'full')
            - kinova_get_wrist_img (hook_id:None, cmd:None)
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

        # if agent_type == "policy_server":
        #     self.pub = rospy.Publisher("/task_cmd", String, queue_size=10)
        #     rospy.Subscriber("/task_status", String, self.task_status_callback)
        # else:
        #     rospy.Subscriber("/task_cmd", String, self.task_cmd_callback)
        #     self.pub = rospy.Publisher("/task_status", String, queue_size=10)


        if self.agent_type == "policy_server":
            self.client = actionlib.SimpleActionClient('/task_action', TaskActionAction)
            print("[policy_server] Waiting for action server...")
            self.client.wait_for_server()
            print("[policy_server] Connected to action server")
        else:
            self.server = actionlib.SimpleActionServer('/task_action', TaskActionAction, self._execute_cb, False)
            self.server.start()
            print(f"[{self.agent_type}] Action server started")

        self._load_modules()
    
        print(f"[{self.agent_type}] Task manager started")

    def _load_modules(self):
        if self.agent_type == "robochair":
            from robochair.kinova_ctrl.kinova_motions.kinova_grasp_strap import Grasp_Strap
            from robochair.wheelchair_ctrl.wheelchair_nav_pc.wheelchair_nav import WC_MotionCommander

            self.kinova_grasp = Grasp_Strap()
            self.wc = WC_MotionCommander()
        elif self.agent_type == "hoyer_sling":
            # TODO: implement loading for hoyer_sling components
            pass
        elif self.agent_type == "policy_server":
            from policy_server.policy_read_yaml import read_policy_yaml
            self.read_policy_yaml = read_policy_yaml

    
    # For agent
    def _execute_cb(self, goal: TaskActionGoal):
        """
        ActionServer callback: receives a task goal, executes it, and returns result.
        """
        rospy.loginfo(f"[{self.agent_type}] Executing task: {goal}")
        task_id = goal.task_id
        hook_id = goal.hook_id
        command = goal.command

        feedback = TaskActionFeedback()
        result = TaskActionResult()

        try:
            executed_cmd, status = self._agent_execute_task(task_id, hook_id, command)

            feedback.feedback = f"Executing {task_id}..."
            self.server.publish_feedback(feedback)

            result.result.success = True if status == "done" else False
            result.result.message = f"task_id:{task_id};cmd:{executed_cmd};status:{status}"
            rospy.loginfo(f"[{self.agent_type}] Done: {result.result.message}")
            self.server.set_succeeded(result.result)

        except Exception as e:
            rospy.logerr(f"[{self.agent_type}] Task error: {e}")
            result.result.success = False
            result.result.message = f"task_id:{task_id};cmd:(INOP.);status:False"
            self.server.set_aborted(result.result)
























    # for policy_server
    def task_status_callback(self, msg):
        if "status:done" in msg.data and self.current_task.split(';')[0] in msg.data:
            print(f"[{self.agent_type}] Received completion: {msg.data}")
            self.task_running = False

    # For agent
    def task_cmd_callback(self, msg):
        rospy.loginfo(f"[{self.agent_type}] Queuing task: {msg.data}")
        self.task_queue.append(msg.data)

    # For agent
    def _agent_handle_task(self, raw_msg):
        try:
            parts = dict(s.split(":") for s in raw_msg.split(";") if ":" in s)
            task_id = parts.get("task_id")
            hook_id = int(parts.get("hook_id", 0))
            command = parts.get("command", "")

            executed_cmd, status = self._agent_execute_task(task_id, hook_id, command)

            status_msg = f"task_id:{task_id};cmd:{executed_cmd};status:{status}"
            self.pub.publish(String(data=status_msg))
            print(f"[{self.agent_type}] Executed: {status_msg}")
            self.task_running = False

        except Exception as e:
            rospy.logerr(f"[{self.agent_type}] Task error: {e}")
            error_msg = f"task_id:{parts.get('task_id', 'UNKNOWN')};cmd:(INOP.);status:False"
            self.pub.publish(String(data=error_msg))
            self.task_running = False

    def _agent_execute_task(self, task_id, hook_id, command):
        if self.agent_type == "robochair":
            # robochair task execution
            if task_id == "kinova_grasp":
                executed_cmd,status = self.kinova_grasp.run(hook_id=hook_id, cmd=command)
                return executed_cmd,status
            elif task_id == "wheelchair_move":
                executed_cmd,status = self.wc.run(hook_id=hook_id, cmd=command)
                return executed_cmd,status 
            elif task_id == "kinova_get_wrist_img":
                return "start_compressed_img", "done"
            elif task_id == "kinova_tie":
                pass
            else:
                rospy.logwarn(f"[{self.agent_type}] Unknown task_id: {task_id}")
                return "(UNKNOWN)", "False"
        
        elif self.agent_type == "hoyer_sling":
            pass
        elif self.agent_type == "bed":
            pass
        
    def run(self):
        if self.agent_type != "policy_server":
            while not rospy.is_shutdown():
                if not self.task_running and self.task_queue:
                    self.current_task = self.task_queue.popleft()
                    self.task_running = True
                    self._agent_handle_task(self.current_task)
        else:
            # load task sequence
            rospack = rospkg.RosPack()
            pkg_path = rospack.get_path('ros_chariot_master')
            path = os.path.join(pkg_path, 'src/policy_server/config/task_seq.yaml')
            task_seq = self.read_policy_yaml(path=path)


            for task in task_seq:
                self.current_task = f"task_id:{task['task_id']};hook_id:{task.get('hook_id', 0)};command:{task.get('command', '')}"

                self.pub.publish(String(data=self.current_task))
                print(f"[{self.agent_type}] Publishing task: {self.current_task}")
                self.task_running = True


                # Wait for task completion signal
                while not rospy.is_shutdown() and self.task_running:
                    rospy.sleep(0.1)


if __name__ == "__main__":
    agent_name = "policy_server"
    rospy.init_node(f"{agent_name}_task_manager")
    manager = TaskManager(agent_name)
    manager.run()