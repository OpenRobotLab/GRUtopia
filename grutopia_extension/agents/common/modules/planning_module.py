import io
import re

import numpy as np
from modules.memory_module import Memory
from modules.vlm.large_model import Vlm_gpt4o
from PIL import Image


class Planning:
    def __init__(self, llm: Vlm_gpt4o, task_config: dict):
        self.large_model = None  # Large language model
        self.llm = llm
        self.task_config = task_config

    def reset(self):
        self.content = None
        self.dialogue_turn = 0
        self.candidates_idx = 0

    def planning(self, observation: dict, memory: Memory):
        # Use large language model for planning
        candidates_true = [
            candidate_idx for candidate_idx, clouds in memory.object_map.clouds.items() if clouds['is_goal'] == 1
        ]
        candidates_possible = [
            candidate_idx for candidate_idx, clouds in memory.object_map.clouds.items() if clouds['is_goal'] == 0
        ]
        if len(candidates_true) > 0 or (
            len(candidates_possible) > 0 and self.dialogue_turn >= self.task_config['dialogue_turn']
        ):
            if len(candidates_true) > 0:
                goal_id = self.choose_candidate(candidates_true, memory)
            elif len(candidates_possible) > 0:
                goal_id = self.choose_candidate(candidates_possible, memory)
            else:
                assert 'Something wrong!'
            nav_goal = memory.object_map.get_best_object(goal_id, observation['position'][:2])
            pick_goal = np.mean(memory.object_map.clouds[goal_id]['clouds'], axis=0)[:3]
            pick_goal_pc = memory.object_map.clouds[goal_id]['clouds']
            self.content = {'goal': nav_goal, 'type': 'candidate', 'goal_3d': pick_goal, 'goal_pc': pick_goal_pc}
            return 'navigation'
        elif len(candidates_possible) > 0 and self.dialogue_turn < self.task_config['dialogue_turn']:
            self.dialogue_turn += 1
            self.ask_question(candidates_possible, memory)
            return 'ask'
        else:
            best_frontier, _ = memory.get_best_frontier(observation)
            self.content = {'goal': best_frontier, 'type': 'frontier'}
            return 'exploration'

    def choose_candidate(self, candidates_idx: list, memory: Memory):
        assert len(candidates_idx) > 0
        if len(candidates_idx) == 1:
            return candidates_idx[0]
        goal_infos = memory.goal_infos
        goal_info = '\n'.join(f"{key}: {' '.join(value)}" for key, value in goal_infos.items() if value)

        candidates_num = len(candidates_idx)
        image_prompt = '\n'.join([f'{i}. See image {i} for candidate_{i}' for i in range(candidates_num)])

        images = []
        for cid in candidates_idx:
            image_buf = io.BytesIO()
            image = Image.fromarray(memory.object_map.clouds[cid]['image'])
            image.save(image_buf, format='PNG')
            images.append(image_buf)

        candidate_id = self.llm.get_answer(
            'choose_candidate', goal=memory.goal, goal_info=goal_info, images=images, image_prompt=image_prompt
        )
        try:
            match = re.search(r'\d+', candidate_id)
            candidate_id = int(match.group()) if match else 0
        except Exception:
            candidate_id = 0
        return candidates_idx[candidate_id]

    def ask_question(self, candidates_idx: list, memory: Memory):
        goal_infos = memory.goal_infos
        goal_info = '\n'.join(f"{key}: {' '.join(value)}" for key, value in goal_infos.items() if value)

        question = None
        # candidates_num = len(candidates_idx)
        # image_prompt = '\n'.join([f'{i}. See image {i} for object_{i}' for i in range(candidates_num)])

        # images = []
        # for cid in candidates_idx:
        #     image_buf = io.BytesIO()
        #     image = Image.fromarray(memory.object_map.clouds[cid]['image'])
        #     image.save(image_buf, format='PNG')
        #     images.append(image_buf)

        # question = self.llm.get_answer('ask_question', goal=memory.goal, goal_info = goal_info, images = images, image_prompt = image_prompt)
        self.question = (
            ('I already know that:' + f'\n{goal_info}' + 'Can you provide more information about the target?')
            if question is None
            else question
        )
