# flake8: noqa
import sys

sys.path.append('GRUtopia/grutopia_extension/agents/mobile_manipulation_agent')

import io
import random
import re

random.seed(2024)
import matplotlib.pyplot as plt
import numpy as np
import shapely
from modules.actuator_module import Actuator
from modules.memory_module import Memory
from modules.planning_module import Planning
from modules.vlm.large_model import Vlm_gpt4o, Vlm_qwen
from PIL import Image
from shapely.geometry import MultiPoint, Point


class MobileManipulationAgent:
    def __init__(self, agent_config: dict):
        task_config = agent_config['task_config']
        self.task_config = task_config

        map_config = agent_config['map_config']

        obstacle_map_config = agent_config['obstacle_map']
        obstacle_map_config['size'] = map_config['size']
        obstacle_map_config['agent_radius'] = map_config['agent_radius']
        obstacle_map_config['pixels_per_meter'] = map_config['pixels_per_meter']

        value_map_config = agent_config['value_map']
        value_map_config['size'] = map_config['size']

        memory_config = {
            'task_config': task_config,
            'map_config': map_config,
            'obstacle_map': obstacle_map_config,
            'value_map': agent_config['value_map'],
            'object_map': agent_config['object_map'],
        }
        actuator_config = {'task_config': task_config, 'map_config': map_config}

        if agent_config['vlm'] == 'gpt':
            self.llm = Vlm_gpt4o(
                azure=True,
                verbose=task_config['verbose'],
                agent_path='GRUtopia/grutopia_extension/agents/mobile_manipulation_agent',
            )
        elif agent_config['vlm'] == 'qwen':
            self.llm = Vlm_qwen(
                verbose=task_config['verbose'],
                agent_path='GRUtopia/grutopia_extension/agents/mobile_manipulation_agent',
            )
        else:
            assert 'Undefined VLM Type!'
        self.memory = Memory(self.llm, memory_config)
        self.planning = Planning(self.llm, task_config)
        self.actuator = Actuator(actuator_config)

        self.stage = 0

        self.very_last_pos = np.array([0, 0])
        self.current_step = 0
        self.very_last_step = 0

    def reset(self, question=None):
        self.question = question
        self.pick_goal, self.pick_goal_description = 'None', 'None'
        self.place_goal, self.place_goal_description = 'None', 'None'
        self.relation = 'None'
        if question:
            task_description = self.llm.get_answer('get_pick_and_place_all', question=question)
            task_descriptions = task_description.strip().split('\n')
            task_descriptions = [td.split(':') for td in task_descriptions]
            for td in task_descriptions:
                if 'pick_goal' == td[0].lower():
                    self.pick_goal = td[1].strip()
                elif 'pick_goal_description' == td[0].lower():
                    self.pick_goal_description = td[1].strip()

                elif 'place_goal' == td[0].lower():
                    self.place_goal = td[1].strip()
                elif 'place_goal_description' == td[0].lower():
                    self.place_goal_description = td[1].strip()

                elif 'relationship' == td[0].lower():
                    self.relation = td[1].strip()

        self.memory.reset(self.pick_goal)
        self.memory.goal_infos['spatial'].append(self.pick_goal_description)
        self.planning.reset()
        self.actuator.reset()

        self.state = 'init_exploration'
        self.dialogue_turn = 0

    def reset_goal(self, goal, goal_description):
        self.memory.reset_goal(goal)
        self.memory.goal_infos['spatial'].append(goal_description)

        self.planning.reset()
        self.actuator.reset()

    def act(self, obs, render):
        memory_render = False
        self.current_step += 1

        if self.current_step - self.very_last_step > 1000:
            if np.linalg.norm(obs['position'][:2] - self.very_last_pos) < 0.1:
                return {'stop': 'None'}
            self.very_last_step = self.current_step
            self.very_last_pos = obs['position'][:2]

        # Update goal info
        if obs.get('answer') is not None:
            self.memory.update_goal_info({'question': self.planning.question, 'answer': obs['answer']})
            self.state = 'planning'

        if render:
            plt.close('all')
            memory_update = self.memory.update(obs)
            memory_render = render and memory_update

            print('state:', self.state)
            print('position:', obs['position'])

        if self.state.split('_')[0] == 'recover':
            self.state = self.state.split('_')[1] if render else self.state
            return {}

        ### INIT EXPLORATION
        if self.stage == 0:  # 'init_exploration'
            finish, action = self.actuator.init_exploration(obs)
            if finish and memory_render:
                self.stage = 1
                self.state = 'pick_planning'
                return {}
            return action

        ### PICK NAV
        elif self.stage == 1:  # 'pick_planning'
            if self.state == 'pick_planning':
                self._update_memory()
                self.state = self.planning.planning(obs, self.memory)
                return {}

            elif self.state == 'exploration' or self.state == 'navigation':
                finish, action = self.actuator.navigate(obs, self.memory, self.planning, render)
                if 'recover' in action:
                    self.state = 'recover_' + self.state
                    return action
                if finish and render:
                    self.state = 'rotate' if finish == 2 else 'pick_planning'
                    return {}
                return action

            elif self.state == 'rotate':  # find the object
                finish, action = self.actuator.rotate(obs, self.memory, self.planning.content['goal'], render)
                if 'recover' in action:
                    self.state = 'recover_' + self.state
                    return action
                if finish and render:
                    self.state = 'DONE' if self.planning.content['type'] == 'candidate' else 'exploration'
                    return {}
                return action

            elif self.state == 'DONE':
                self.stage = 2
                self.state = 'move_arm'
                return {'find_pick': self.planning.content['goal_3d']}

            else:
                assert 'Planning an undefined action'

        ### PICK
        elif self.stage == 2:
            if self.state == 'move_arm':
                finish, action = self.actuator.arm_ik(obs, self.planning.content['goal_3d'])
                if finish and render:
                    self.state = 'grasp'
                    return {}
                return action
            elif self.state == 'grasp':
                if render:
                    self.state = 'reset_goal'
                    return {'gt_grasp': {}}
                return {}
            elif self.state == 'reset_goal':
                if render:
                    self.stage = 3
                    self.state = 'place_planning'
                    self.reset_goal(self.place_goal, self.place_goal_description)
                return {}

        ### PLACE NAV
        elif self.stage == 3:
            if self.state == 'place_planning':
                self._update_memory()
                self.state = self.planning.planning(obs, self.memory)

            if self.state == 'exploration' or self.state == 'navigation':
                finish, action = self.actuator.navigate(obs, self.memory, self.planning, render)
                if 'recover' in action:
                    self.state = 'recover_' + self.state
                    return action
                if finish and render:
                    self.state = 'rotate' if finish == 2 else 'place_planning'
                    return {}
                return action

            elif self.state == 'rotate':  # find the object
                finish, action = self.actuator.rotate(obs, self.memory, self.planning.content['goal'], render)
                if 'recover' in action:
                    self.state = 'recover_' + self.state
                    return action
                if finish and render:
                    self.state = 'DONE' if self.planning.content['type'] == 'candidate' else 'exploration'
                    return {}
                return action

            elif self.state == 'DONE':
                self.stage = 4
                self.state = 'check_relation'
                return {}

            else:
                assert 'Planning an undefined action'

        ### PLACE
        elif self.stage == 4:
            if self.state == 'check_relation':
                if self.relation == 'on':
                    point_clouds = self.planning.content['goal_pc']
                    goal_polygon = MultiPoint(point_clouds[:, :2]).convex_hull

                    reachable_polygon = Point(obs['position'][:2]).buffer(1)
                    place_polygon = shapely.intersection(goal_polygon, reachable_polygon)
                    if place_polygon is not None:
                        self.place_goal_position = np.array(
                            [place_polygon.centroid.x, place_polygon.centroid.y, point_clouds[:, 2].max()]
                        ) + np.array([0, 0, 0.1])

                        self.state = 'move_arm'
                        return {}
                    else:
                        return {'gt_loose': 'none'}

                elif self.relation in ['near', 'nearby']:
                    return {'gt_loose': 'end'}

                else:
                    return {'gt_loose': 'end'}

            elif self.state == 'move_arm':
                finish, action = self.actuator.arm_ik(obs, self.place_goal_position)
                if finish and render:
                    # self.state = 'wait_fall'
                    # return {'gt_loose': 'loose'}
                    return {'gt_loose': 'end'}
                return action

        assert 'Undefined type'

    def _update_memory(self):
        candidates_idx = [
            candidate_id
            for candidate_id, clouds in self.memory.object_map.clouds.items()
            if clouds.get('is_goal') == None
        ]
        for candidate_id in candidates_idx:
            # Use large language model for planning
            goal_infos = self.memory.goal_infos
            goal_info = '\n'.join(f"{key}: {' '.join(value)}" for key, value in goal_infos.items() if value)

            images = self.memory.object_map.clouds[candidate_id]['images']
            if len(images) < 4:
                selected_images = [image['image'] for image in images]
                remaining_count = 4 - len(selected_images)
                selected_images.extend(random.choices(selected_images, k=remaining_count))
            else:
                sorted_images = sorted(images, key=lambda x: x['number'], reverse=True)
                selected_images = [image['image'] for image in sorted_images[:4]]

            # Merge all the 4 images as one image (2x2 grid)
            top_row = np.concatenate([np.array(selected_images[0]), np.array(selected_images[1])], axis=1)
            bottom_row = np.concatenate([np.array(selected_images[2]), np.array(selected_images[3])], axis=1)
            concated_image = np.concatenate([top_row, bottom_row], axis=0)
            self.memory.object_map.clouds[candidate_id]['image'] = concated_image

            image_buf = io.BytesIO()
            image = Image.fromarray(concated_image)
            image.save(image_buf, format='PNG')
            candidates_type = self.llm.get_answer(
                'check_candidates', goal=self.memory.goal, label=candidate_id, goal_info=goal_info, images=[image_buf]
            )
            try:
                match = re.search(r'\d+', candidates_type)
                candidates_type = int(match.group()) if match else 0
            except Exception:
                candidates_type = 0
            self.memory.object_map.clouds[candidate_id]['is_goal'] = candidates_type
