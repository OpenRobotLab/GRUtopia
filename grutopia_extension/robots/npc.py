from collections import OrderedDict
from typing import Dict

from grutopia.core.config.robot import RobotCfg
from grutopia.core.robot.robot import BaseRobot
from grutopia.core.scene.scene import IScene
from grutopia.core.util import log


@BaseRobot.register('NPC')
class NPC(BaseRobot):
    def __init__(self, robot_model: RobotCfg, scene: IScene):
        super().__init__(robot_model, scene)
        self.name_of_robots_in_scene = []
        if robot_model.prim_path is not None:
            # TODO implement when NPCs need a body.
            log.info('NPC has a body, but this is not implemented yet.')
        log.debug(f'NPC {self.name} initialized')

    def set_up_to_scene(self, scene: IScene):
        # No need to add to scene as this is a non-prim robot
        pass

    def post_reset(self):
        log.debug(f'Post-reset for NPC {self.name}')
        pass

    def apply_action(self, action: Dict):
        log.debug(f'Applying action for NPC {self.name}: {action}')
        pass

    def get_obs(self) -> OrderedDict:
        log.debug(f'Getting observation for NPC {self.name}')
        # npc can get all the robot obs in the scene, but this method returns nothing.
        return self._make_ordered()
