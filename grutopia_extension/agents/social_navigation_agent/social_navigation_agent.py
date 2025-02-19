import sys

sys.path.append('GRUtopia/grutopia_extension/agents/social_navigation_agent')
import io
import random
import re

random.seed(2024)
import matplotlib.pyplot as plt
import numpy as np
from modules.actuator_module import Actuator
from modules.memory_module import Memory
from modules.planning_module import Planning
from modules.vlm.large_model import Vlm_gpt4o, Vlm_qwen
from PIL import Image


class SocialNavigationAgent:
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
            self.llm = Vlm_gpt4o(azure=True, verbose=task_config['verbose'])
        elif agent_config['vlm'] == 'qwen':
            self.llm = Vlm_qwen(verbose=task_config['verbose'])
        else:
            assert 'Undefined VLM Type!'
        self.memory = Memory(self.llm, memory_config)
        self.planning = Planning(self.llm, task_config)
        self.actuator = Actuator(actuator_config)

    def reset(self, question=None):
        self.question = question
        if question:
            goal = self.llm.get_answer('get_goal', question=question)
            if self.task_config['verbose']:
                print(f'The goal of this episode is {goal}')
            self.memory.reset(goal)
            self.memory.update_goal_info({'question': question, 'answer': 'OK.'})
        else:
            self.memory.reset('bed')
            self.memory.goal_infos['spatial'].append('The goal is located near a cabinet.')
        self.planning.reset()
        self.actuator.reset()
        self.state = 'init_exploration'

        self.dialogue_turn = 0

    def act(self, obs, render):
        memory_render = False

        # Update goal info
        if obs.get('answer') is not None:
            self.memory.update_goal_info({'question': self.planning.question, 'answer': obs['answer']})
            self.state = 'planning'

        if render:
            plt.close('all')
            memory_update = self.memory.update(obs)
            memory_render = render and memory_update

        if self.state.split('_')[0] == 'recover':
            self.state = self.state.split('_')[1] if render else self.state
            return {}

        if self.state == 'init_exploration':
            finish, action = self.actuator.init_exploration(obs)
            if finish and memory_render:
                self.state = 'planning'
                return {}
            return action
        elif self.state == 'planning':
            self._update_memory()
            self.state = self.planning.planning(obs, self.memory)
            if self.state == 'ask':
                return {'web_chat': self.planning.question}
            elif self.state == 'exploration' or self.state == 'navigation' or self.state == 'DONE':
                return {}
            else:
                assert 'Planning an undefined action'
        elif self.state == 'exploration' or self.state == 'navigation':
            finish, action = self.actuator.navigate(obs, self.memory, self.planning, render)
            if 'recover' in action:
                self.state = 'recover_' + self.state
                return action
            if finish and render:
                self.state = 'rotate' if finish == 2 else 'planning'
                return {}
            return action
        elif self.state == 'rotate':
            finish, action = self.actuator.rotate(obs, self.memory, self.planning.content['goal'], render)
            if 'recover' in action:
                self.state = 'recover_' + self.state
                return action
            if finish and render:
                self.state = 'DONE' if self.planning.content['type'] == 'candidate' else 'exploration'
                return {}
            return action
        elif self.state == 'ask':
            return {}
        elif self.state == 'DONE':
            return {'finish': {}}

        assert 'Undefined type'

    def _update_memory(self):
        candidates_idx = [
            candidate_id
            for candidate_id, clouds in self.memory.object_map.clouds.items()
            if clouds.get('is_goal') is None
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
                'check_candidates',
                goal=self.memory.goal,
                label=candidate_id,
                goal_info=goal_info,
                images=[image_buf],
            )
            try:
                match = re.search(r'\d+', candidates_type)
                candidates_type = int(match.group()) if match else 0
            except Exception as e:
                print(e)
                candidates_type = 0
            self.memory.object_map.clouds[candidate_id]['is_goal'] = candidates_type
