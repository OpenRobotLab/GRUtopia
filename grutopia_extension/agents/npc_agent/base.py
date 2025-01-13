from threading import Thread
from typing import Dict, List, Union

from grutopia.core.util import log
from grutopia_extension.agents.npc_agent.config import NPCUserConfig
from grutopia_extension.agents.npc_agent.llm_caller import LLMCaller, run_llm_caller
from grutopia_extension.agents.npc_agent.utils import Message


class NPC:
    def __init__(self, config: NPCUserConfig, extra: Dict) -> None:
        self.config = config
        self.caller = LLMCaller(
            extra['npc_scene_data_config'],
            config.max_interaction_turn,
            config.model_name,
            config.openai_api_key,
            config.api_base_url,
        )
        self.processed_user_message = 0
        self.thread = Thread(target=run_llm_caller, args=(self.caller,))
        self.thread.daemon = True
        self.thread.start()

    def feed(self, npc_name: str, obs: Union[Dict, None], unread_chat_list: List[Dict]) -> Union[List[Message], None]:
        """feed npc with observation.

        Args:
            npc_name (str): npc name
            obs (dict): full observation of the world, with hierarchy of
              obs
                task_name:
                  robot_name:
                    position
                    orientation
                    controller_0
                    controller_1
                    ...
                    sensor_0
                    sensor_1
                    ...
            unread_chat_list (list): list of unread chat messages.

        Returns:
            Union[List, None]: List of responses.

        """
        if obs is None:
            return []
        response_list = []
        for message in unread_chat_list:
            if npc_name in message['at']:
                robot_obs = obs[message['name']]
                position = robot_obs.get('position', None)
                orientation = robot_obs.get('orientation', None)

                bbox_label_data = robot_obs['camera']['bounding_box_2d_tight']
                bbox = bbox_label_data['data']
                idToLabels = bbox_label_data['info']['idToLabels']
                # for message in user_messages:
                log.info(f'send message: {message}')
                self.caller.infer(message)
                self.processed_user_message += 1
                self.caller.update_robot_view(bbox, idToLabels)
                self.caller.update_robot_pose(position, orientation)
        if not self.caller.response_queues.empty():
            response_list.append(self.caller.response_queues.get())
        return response_list
