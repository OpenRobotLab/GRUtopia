from typing import Dict, List, Optional

import yaml

from grutopia.core.config import AgentConfig, EpisodeConfigFile, SimConfig, ValidatedConfig
from grutopia.core.datahub import DataHub
from grutopia.core.runtime.task_runtime import TaskRuntime, TaskRuntimeManager
from grutopia.core.util import log


class SimulatorRuntime:
    """SimulatorRuntime"""

    def __init__(self, config_path: str, headless: bool = True, webrtc: bool = False, native: bool = False):
        """

        Args:
            config_path: config file (yaml) path
        """
        self.env_num = 1
        self.config_file_path = config_path
        self.config = None
        self.simulator: Optional[SimConfig] = None
        self.task_runtime_manager: Optional[TaskRuntimeManager] = None
        self.agents: Optional[List[AgentConfig]] = []
        self.init(self.get_config_from_file())

        # Init Isaac Sim
        from isaacsim import SimulationApp  # noqa
        self.headless = headless
        self._simulation_app = SimulationApp({'headless': self.headless, 'anti_aliasing': 0})

        if webrtc:
            from omni.isaac.core.utils.extensions import enable_extension  # noqa

            self._simulation_app.set_setting('/app/window/drawMouse', True)
            self._simulation_app.set_setting('/app/livestream/proto', 'ws')
            self._simulation_app.set_setting('/app/livestream/websocket/framerate_limit', 60)
            self._simulation_app.set_setting('/ngx/enabled', False)
            enable_extension('omni.services.streamclient.webrtc')

        elif native:
            from omni.isaac.core.utils.extensions import enable_extension  # noqa

            self._simulation_app.set_setting('/app/window/drawMouse', True)
            self._simulation_app.set_setting('/app/livestream/proto', 'ws')
            self._simulation_app.set_setting('/app/livestream/websocket/framerate_limit', 120)
            self._simulation_app.set_setting('/ngx/enabled', False)
            enable_extension('omni.kit.streamsdk.plugins-3.2.1')
            enable_extension('omni.kit.livestream.core-3.2.0')
            enable_extension('omni.kit.livestream.native')

    @property
    def simulation_app(self):
        return self._simulation_app

    @staticmethod
    def read_yaml_file(file_path) -> Dict:
        if file_path:
            if not file_path.endswith('.yaml') or \
                    file_path.endswith('.yml'):
                log.error('runtime file not end with .yaml or .yml')
                raise FileNotFoundError('runtime file not end with .yaml or .yml')
            with open(file_path, 'r') as f:
                config_dict = yaml.load(f.read(), yaml.FullLoader)
            return config_dict
        log.error('Config file path is not set')
        raise FileNotFoundError('Config file path is not set')

    def get_config_from_file(self):
        self.config = self.read_yaml_file(self.config_file_path)
        return self.config

    def init(self, config_dict: Dict) -> None:
        config = ValidatedConfig(**config_dict)

        # Load Episodes file if episodes in task_config is instance of str.
        if isinstance(config.task_config.episodes, str):
            episodes_dict = self.read_yaml_file(config.task_config.episodes)
            config.task_config.episodes = EpisodeConfigFile(**episodes_dict).episodes

        # Init Datahub
        DataHub.datahub_init(sim=str(config.datahub_config.sim),
                             chat=str(config.datahub_config.chat),
                             remote=config.datahub_config.remote)

        _trm = TaskRuntimeManager(config.task_config)
        self.simulator = config.simulator
        self.task_runtime_manager = _trm
        self.env_num = _trm.env_num
        self.agents = config.agents
