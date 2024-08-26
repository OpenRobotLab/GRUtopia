from typing import Any, Dict, List, Union

import numpy as np
from omni.isaac.core.scenes import Scene
from omni.isaac.core.utils.types import ArticulationAction

from grutopia.core.datahub import DataHub
from grutopia.core.robot.controller import BaseController
from grutopia.core.robot.robot import BaseRobot
from grutopia.core.robot.robot_model import ControllerModel


@BaseController.register('WebChatboxController')
class WebChatboxController(BaseController):
    """Controller for interacting with webui chatbox."""

    def __init__(self, config: ControllerModel, robot: BaseRobot, scene: Scene, npc: bool = False) -> None:
        """Initialize the controller.

        Args:
            config (ControllerModel): merged config (from user config and robot model) of the controller.
            robot (BaseRobot): robot owning the controller.
            scene (Scene): scene from isaac sim.
            npc (bool, optional): chat as npc or not. Defaults to False.
        """
        super().__init__(config, robot, scene)

        self._role = 'user'
        if npc:
            self._role = 'agent'
        self._nickname = robot.name
        self.log_data = None
        self.chat_control = None

    def action_to_control(self, action: Union[np.ndarray, List]) -> ArticulationAction:
        assert len(action) == 1, 'action must contain 1 elements'
        return self.forward(text=str(action[0]))

    def forward(self, text: str) -> ArticulationAction:
        DataHub.send_chat_control(nickname=self._nickname, text=text, img='', role=self._role)
        return ArticulationAction()

    def get_obs(self) -> Dict[str, Any]:
        chat_control = DataHub.get_chat_control()
        log_data = DataHub.get_log_data()
        if chat_control:
            self.chat_control = chat_control
        if log_data:
            self.log_data = log_data
        return {
            'chat_control': self.chat_control,
            'log_data': self.log_data,
        }
