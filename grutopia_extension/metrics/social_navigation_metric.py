import json
from typing import List

import numpy as np

from grutopia.core.config.metric import MetricUserConfig
from grutopia.core.task.metric import BaseMetric

# from grutopia.core.util import log


@BaseMetric.register('SocialNavigationMetric')
class SocialNavigationMetric(BaseMetric):

    def __init__(self, config: MetricUserConfig):
        super().__init__(config)
        self.dialogue_landmarks = None
        self.dialogue = None
        self.path = None
        self.candidates_reduced = None
        self.original_candidates = None
        self.dialogue_turn = None
        self.fall_times = None
        self.path_length = None
        self.current_position = None
        self.total_episodes = 0
        self.reset()

    def reset(self):
        self.path = None
        self.current_position = None
        self.dialogue = []
        self.dialogue_landmarks = {}
        self.candidates_reduced = []
        self.original_candidates = 0
        self.dialogue_turn = 0
        self.fall_times = 0
        self.path_length = 0

    def update(self, obs: dict):
        # TODO result结构
        # 每一步
        pass

    def calc_one_episode(self, result_dict: dict, task_info: dict, save_path: str):
        """

        """
        self.total_episodes += 1
        self.path_lengths.append(self.path_length)
        self.rts.append(self.fall_times)
        navigation_error = np.linalg.norm(np.array(self.current_position[:2]) - task_info['position'][:2])
        self.navigation_errors.append(navigation_error)
        result_dict['success'] = result_dict['success_view'] and (navigation_error <= 3)
        self.successes.append(result_dict['success'])
        self.spls.append(result_dict['success'] * task_info['shortest_path_length'] /
                         max(self.path_length, task_info['shortest_path_length']))
        self.ecrs.append(np.average(self.candidates_reduced) if len(self.candidates_reduced) > 0 else 0)
        self.dialogue_turns.append(self.dialogue_turn)

        result_dict['dialogue_turn'] = self.dialogue_turn
        result_dict['dialogue'] = self.dialogue
        result_dict['dialogue_landmarks'] = self.dialogue_landmarks
        result_dict['candidates_reduced'] = self.candidates_reduced
        result_dict['original_candidates'] = self.original_candidates
        result_dict['fall_times'] = self.fall_times
        result_dict['path_length'] = self.path_length
        result_dict['last_position'] = self.current_position

        assert len(self.path_lengths) == self.total_episodes
        assert len(self.navigation_errors) == self.total_episodes
        assert len(self.successes) == self.total_episodes
        assert len(self.spls) == self.total_episodes
        assert len(self.dialogue_turns) == self.total_episodes
        assert len(self.ecrs) == self.total_episodes

        json_data = json.dumps(result_dict, indent=4)
        with open(save_path, 'w') as file:
            file.write(json_data)
        print('\n', result_dict)

    @staticmethod
    def _calc(param: List):
        return np.average(param) if len(param) > 0 else 0

    def calc(self):
        """在runner中执行，在这里实现

        (((1 + 1) + 1) + 1)

        (1 + 1 + 1 + 1)

        """
        avg_path_length = self._calc(self.path_lengths)
        avg_nav_error = self._calc(self.navigation_errors)
        sr = self._calc(self.successes)
        rt = self._calc(self.rts)
        spl = self._calc(self.spls)
        avg_dialogue_turns = self._calc(self.dialogue_turns)
        avg_candidates_reduction = self._calc(self.candidates_reduced)
        print({
            'SR': sr,
            'PL': avg_path_length,
            'NE': avg_nav_error,
            'SPL': spl,
            'DT': avg_dialogue_turns,
            'ECR': avg_candidates_reduction,
            'RT': rt
        })
