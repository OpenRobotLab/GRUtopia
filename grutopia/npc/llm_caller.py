from queue import Queue
from typing import Dict, List, Literal, Tuple

import numpy as np
import requests

from grutopia.core.util import log
from grutopia.npc.prompt import in_context_example, system_message
from grutopia.npc.utils import Env


class LLMCaller:

    def __init__(
        self,
        scene_data_path: str,
        max_interaction_turn: int,
        model_name: str,
        openai_api_key: str,
        api_base_url: str,
    ) -> None:
        self.model_name = model_name
        self.request_queue = Queue()
        self.response_queues = Queue()
        self.env = Env(scene_data_path)
        self.system_message = system_message
        self.max_turn = max_interaction_turn
        self.header = {'Authorization': f'Bearer {openai_api_key}'}
        self.api_base_url = api_base_url
        self.history_messages = []

    def _construct_init_messages(self, question: str) -> List[dict]:
        messages = [{
            'role': 'system',
            'content': [{
                'type': 'text',
                'text': self.system_message
            }]
        }, {
            'role': 'user',
            'content': [{
                'type': 'text',
                'text': question
            }]
        }, {
            'role': 'assistant',
            'content': [{
                'type': 'text',
                'text': in_context_example
            }]
        }]
        code_result, _ = self._parse_and_execution(in_context_example)
        messages.append({'role': 'user', 'content': [{'type': 'text', 'text': code_result}]})
        messages.extend(self.history_messages)
        return messages

    def _parse_and_execution(self, response: str) -> Tuple[str, bool]:
        if '```python' in response:
            code = response.split('```python')[1].split('```')[0].strip()
            result = self.env(code)
            return f'Code execution result:\n{result}\n', False
        elif '[answer]' in response.lower():
            return response.lower().split('[answer]')[1].split('[/answer]')[0], True
        else:
            return 'Continue. If you need some information, write code to get it, wrap your code in ```python and ```. If you have got the final answer, tell the answer in the format: [answer]YOUR FINAL ANSWER[/answer].', False

    def _infer(self, question: str) -> str:
        """Given the facing-to object's ID and question, generate the response by LLM.

        Args:
            question (str): The question to ask.
            facing_to_object_id (Optional[str]): The ID of the object the jetbot is facing to. Defaults to None.

        Returns:
            str: The response from LLM.
        """
        log.info(f'construct messages for question: {question}')
        messages = self._construct_init_messages(question)
        for _ in range(self.max_turn):
            payload = {'model': self.model_name, 'messages': messages}
            try:
                response = requests.post(self.api_base_url, headers=self.header, json=payload).json()
                message = response['choices'][0]['message']
            except KeyError:
                log.info(f'Got an unexpected result when calling openai api: {response}')
                return 'Got an unexpected result when calling openai api.'
            except Exception as e:
                return f'Got an exception when calling openai api: {e}'
            messages.append(message)
            result, is_end = self._parse_and_execution(message['content'])
            if is_end:
                self.history_messages.extend([{
                    'role': 'user',
                    'content': [{
                        'type': 'text',
                        'text': question
                    }]
                }, {
                    'role': 'assistant',
                    'content': [{
                        'type': 'text',
                        'text': result
                    }]
                }])
                return result
            messages.append({'role': 'user', 'content': [{'type': 'text', 'text': result}]})

        return 'Sorry, I cannot answer this question.'

    def infer(self, question: str) -> None:
        self.request_queue.put(question)

    def update_robot_view(self, bbox: np.ndarray, idToLabels: Dict[str, Dict[Literal['class'], str]]) -> None:
        """Get the face-to object ID by 2D bounding box.
        Args:
            bbox (np.array): The 2D bounding box in [x_min, y_min, x_max, y_max] format.
            idToLabels (Dict[str, Dict[Literal["class"], str]]): The mapping from ID to labels.
        """
        # clear the all_objects_in_view list
        code = """all_objects_in_view.clear()"""
        self.env(code)
        for row_idx in range(bbox.shape[0]):
            _id = str(bbox[row_idx][0])
            semantic_label = idToLabels[_id]['class']
            sub_bbox = np.array(
                [int(bbox[row_idx][1]),
                 int(bbox[row_idx][2]),
                 int(bbox[row_idx][3]),
                 int(bbox[row_idx][4])])
            self._process_2d_bbox(semantic_label, sub_bbox)

    def _process_2d_bbox(self, label: str, bbox: List[float]) -> None:
        """Get the center, size of 2d bbox in view, update the corresponding variable in code execution env.
        Args:
        label: str: The label of the object.
        bbox: List[float]: The 2D bounding box in [x_min, y_min, x_max, y_max] format.
        """
        center = (bbox[:2] + bbox[2:4]) / 2
        size = bbox[2:4] - bbox[:2]
        code = f"""all_objects_in_view.append(ObjectInView(id="{label}", center=np.array([{center[0]}, {center[1]}]), size=np.array([{size[0]}, {size[1]}])))"""
        self.env(code)

    def update_robot_pose(self, position: np.ndarray, orientation: np.ndarray) -> None:
        """Update the current position and orientation of the robot.
        Args:
            position (np.ndarray): The position of the robot.
            orientation (np.ndarray): The orientation of the robot.
        """
        code = f"""
current_position = {list(position)}
current_orientation = {list(orientation)}"""
        self.env(code)


def run_llm_caller(caller: LLMCaller):
    while True:
        prompt = caller.request_queue.get()
        try:
            response = caller._infer(prompt)
            caller.response_queues.put(response)
        except Exception as e:
            log.error(f'Exception occurred in llm infer when process prompt={prompt}: {e}')
            caller.response_queues.put('Exception occurred, please check the log.')
