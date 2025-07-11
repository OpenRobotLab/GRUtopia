from typing import List, Optional

from grutopia.core.config import EpisodeCfg
from grutopia.core.config.robot import RobotCfg
from grutopia.core.config.task import TaskCfg
from grutopia.core.robot.robot import BaseRobot
from grutopia.core.runtime.task_runtime import TaskRuntime
from grutopia.core.scene.scene import IScene
from grutopia.core.task import BaseTask
from grutopia.macros import gm
from grutopia_extension.robots.h1 import H1Robot


class MultiSimTestH1RobotCfg(RobotCfg):
    name: Optional[str] = 'h1'
    type: Optional[str] = 'MultiSimTestH1Robot'
    prim_path: Optional[str] = '/h1'
    create_robot: Optional[bool] = True
    usd_path: Optional[str] = gm.ASSET_PATH + '/robots/h1/h1.usd'


class MultiSimTestEpisodeCfg(EpisodeCfg):
    pass


class MultiSimTestTaskCfg(TaskCfg):
    type: Optional[str] = 'MultiSimTestTask'
    episodes: List[MultiSimTestEpisodeCfg]


@BaseRobot.register('MultiSimTestH1Robot')
class MultiSimTestH1Robot(H1Robot):
    def __init__(self, config: MultiSimTestH1RobotCfg, scene: IScene):
        super().__init__(config, scene)
        self.current_action = None
        self.action_list = []

    def apply_action(self, action: dict):
        self.current_action = action
        self.action_list.append(action['test_controller'])

    def get_obs(self):
        obs = {'observed_actions': self.action_list}
        return self._make_ordered(obs)


@BaseTask.register('MultiSimTestTask')
class MultiSimTestTask(BaseTask):
    def __init__(self, runtime: TaskRuntime, scene: IScene):
        super().__init__(runtime, scene)
        self.episode_id = runtime.extra['episode_id']
        self.stop_count = 0
        self.max_step = 50

    def get_observations(self):
        obs = super().get_observations()
        obs['h1']['episode_id'] = self.episode_id
        return obs

    def is_done(self):
        self.stop_count += 1
        return self.stop_count >= self.max_step
