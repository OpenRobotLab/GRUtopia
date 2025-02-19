import json
import os
from typing import Dict

import numpy as np
import yaml
from scipy.spatial import KDTree

from grutopia.core.datahub.datahub import ActionData
from grutopia.core.runtime.task_runtime import TaskRuntime
from grutopia.core.util import log
from grutopia_extension.agents.core.agent import BaseAgent


@BaseAgent.register('MobileManipulationAgent')
class MobileManipulationAgentClient(BaseAgent):
    def __init__(
        self, task_name: str, robot_name: str | None, agent_config: Dict, sync_mode: str, task_runtime: TaskRuntime
    ):
        super().__init__(task_name, robot_name, agent_config, sync_mode, task_runtime)
        self.term = False
        log.debug(f'=============== agent_config: {agent_config} ===============')

        from grutopia_extension.agents.mobile_manipulation_agent.mobile_manipulation_agent import (
            MobileManipulationAgent,
        )

        with open('./GRUtopia/grutopia_extension/agents/mobile_manipulation_agent/agent_config.yaml', 'r') as file:
            agent_config = yaml.safe_load(file)
        self.agent = MobileManipulationAgent(agent_config)
        self.agent.reset(self.task_runtime.extra['instruction'])

        self.wrong_message = '[Nav1]Can not find pick object!'
        self.pick_message = {}

        # Construct a KDTree for given category to make gt grasp
        # HC by JH: Read metadata (may not be accessed by mm api)
        self.extra = self.task_runtime.extra
        with open(os.path.join(self.extra['meta_path'], 'object_dict.json'), 'r') as file:
            self.object_dict = json.load(file)
        self.gt_goal_instance_id_candidates = self.extra['candidates']

        pick_goal = self.agent.pick_goal
        self.pick_goal_category_keys = [
            obj_key for obj_key, obj_value in self.object_dict.items() if obj_value['category'] == pick_goal
        ]
        self.pick_goal_kdtree = KDTree(
            np.array([self.object_dict[key]['position'] for key in self.pick_goal_category_keys]) * 0.01
        )  # HC by JH: scale of scene

    def decision_making(self, obs) -> Dict:
        if obs['sim_step'] > self.task_runtime.task_settings.max_step:
            self.terminate()
        chat_control = self.chat.get_message()
        if len(chat_control) > 0 and chat_control[0]['name'] == 'npc' + '_' + str(self.task_runtime.env.env_id):
            obs['answer'] = chat_control[0]['message']
            self.set_observation(obs)
        render = obs.pop('render')
        agent_return = self.agent.act(obs, render=render)
        # agent_return = {'finish': {}}
        controllers = []
        while len(agent_return) > 0:
            controller, data = agent_return.popitem()
            if controller == 'finish':
                if render:
                    self.terminate()
                continue
            elif controller == 'stop':
                self.terminate()
            elif controller == 'find_pick':
                # Get the closest object from KDTree of given category
                pick_goal_position = data
                _, pick_goal_prim_index = self.pick_goal_kdtree.query(pick_goal_position)
                pick_goal_instance_id = self.object_dict[self.pick_goal_category_keys[pick_goal_prim_index]][
                    'instance_id'
                ]
                pick_goal_prim_path = (
                    '/World/env_0/scene/Meshes/'
                    + self.object_dict[self.pick_goal_category_keys[pick_goal_prim_index]]['scope']
                    + '/'
                    + pick_goal_instance_id
                )  # HC by JH: not sure how to access the path
                self.pick_goal_prim_path = pick_goal_prim_path

                # Check if the pick goal is correct
                choose_flag = False
                for gt_goal_instance_id in self.gt_goal_instance_id_candidates:
                    if pick_goal_instance_id == gt_goal_instance_id:
                        gt_goal_position = self.object_dict[gt_goal_instance_id]['position']
                        choose_flag = True
                        break

                if choose_flag:
                    pick_distance = np.linalg.norm(gt_goal_position - pick_goal_position)
                    self.pick_message['pick_distance'] = pick_distance
                    if pick_distance > 0.5:
                        self.wrong_message = f'[Pick]coincidentally recognize as the right object!({pick_distance})'
                        self.terminate()
                    elif pick_distance > 0.1:
                        self.wrong_message = f'[Pick]Grasp position is not accurate!({pick_distance})'
                        self.terminate()
                    else:
                        self.wrong_message = f'[Pick]Find the right object to pick!{pick_distance}'
                        import omni
                        from omni.isaac.core.prims import RigidPrim

                        self.pick_goal_rigid = RigidPrim(pick_goal_prim_path, scale=[0.2, 0.2, 0.2])
                        self.pick_goal_rigid.set_mass(1e-6)

                        # Fix the robot to the ground for picking
                        robot_prim_path = '/World/env_0/robots/h1_with_hand'
                        from grutopia.core.util.joint import create_joint

                        create_joint(
                            prim_path=robot_prim_path + '/fix_joint',
                            joint_type='FixedJoint',
                            body0=robot_prim_path + '/torso_link',
                            enabled=True,
                        )

                else:
                    self.pick_message['pick_object'] = pick_goal_instance_id
                    self.wrong_message = '[Pick]Choose the wrong object!'
                    self.terminate()

            elif controller == 'gt_grasp':
                from grutopia.core.util.physics import deactivate_collider

                deactivate_collider(omni.isaac.core.utils.prims.get_prim_at_path(self.pick_goal_prim_path))

                robot_prim_path = '/World/env_0/robots/h1_with_hand'
                from grutopia.core.util.joint import create_joint

                create_joint(
                    prim_path=self.pick_goal_prim_path + '/pick_joint',
                    joint_type='FixedJoint',
                    body0=self.pick_goal_prim_path,
                    body1=robot_prim_path + '/right_hand_link',
                    enabled=True,
                )

                create_joint(prim_path=robot_prim_path + '/fix_joint', joint_type='FixedJoint', enabled=False)

                self.wrong_message = '[Nav2]Can not find place object!'

            elif controller == 'gt_loose':
                if data == 'none':
                    self.wrong_message = '[Place]No appropriate point to place!'
                    self.terminate()

                elif data == 'end':
                    self.wrong_message = '[DONE]The agent complete the task!'
                    self.terminate()

            controllers.append({'controller': controller, 'data': data})
        action = ActionData(**{'robot': self.robot_name, 'controllers': controllers})
        self._set_actions({self.task_name: action})
        if self.term:
            return {'terminate': True}
        return {c['controller']: c['data'] for c in controllers}

    def terminate(self):
        """
        Set task finish for task with self.task_name to Datahub.
        Called in self.decision_making by users.
        """
        log.info(f'Task {self.task_name} finished')
        self.term = True
