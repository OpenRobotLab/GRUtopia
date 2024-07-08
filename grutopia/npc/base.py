from threading import Thread

from grutopia.core.config import RobotUserConfig
from grutopia.core.config.npc import NPCUserConfig
from grutopia.core.robot.controller import BaseController
from grutopia.core.robot.robot import BaseRobot
from grutopia.core.robot.robot_model import ControllerModel
from grutopia.core.util import log
from grutopia.npc.llm_caller import LLMCaller, run_llm_caller


class NPC:

    def __init__(self, config: NPCUserConfig) -> None:
        self.config = config
        web_chat_config = ControllerModel(
            name='web_chat',
            type='WebChatboxController',
        )
        npc_config = RobotUserConfig(
            name='NPC',
            type='npc',
            prim_path='/npc',
        )
        npc = BaseRobot(config=npc_config, robot_model=None, scene=None)
        self.web_chat = BaseController.controllers['WebChatboxController'](config=web_chat_config,
                                                                           robot=npc,
                                                                           scene=None,
                                                                           npc=True)
        self.caller = LLMCaller(config.scene_data_path, config.max_interaction_turn, config.model_name,
                                config.openai_api_key, config.api_base_url)
        self.processed_user_message = 0
        self.thread = Thread(target=run_llm_caller, args=(self.caller, ))
        self.thread.daemon = True
        self.thread.start()

    def feed(self, obs: dict):
        """feed npc with observation.

        Args:
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
        """
        for task_obs in obs.values():
            for robot_obs in task_obs.values():
                chat = robot_obs.get('web_chat', None)
                if chat is not None and chat['chat_control']:
                    position = robot_obs.get('position', None)
                    orientation = robot_obs.get('orientation', None)
                    user_messages = []
                    for message in chat['chat_control']:
                        if message['type'] == 'user':
                            user_messages.append(message['message'])
                    user_messages = user_messages[self.processed_user_message:]
                    bbox_label_data = robot_obs['camera']['frame']['bounding_box_2d_tight']
                    bbox = bbox_label_data['data']
                    idToLabels = bbox_label_data['info']['idToLabels']
                    for message in user_messages:
                        log.info(f'send message: {message}')
                        self.caller.infer(message)
                        self.processed_user_message += 1
                        self.caller.update_robot_view(bbox, idToLabels)
                        self.caller.update_robot_pose(position, orientation)
                    if not self.caller.response_queues.empty():
                        response = self.caller.response_queues.get()
                        self.web_chat.action_to_control([response])
