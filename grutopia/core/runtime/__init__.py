from typing import Dict, Optional

import yaml

from grutopia.core.config import Config, EpisodeConfigFile, SimConfig
from grutopia.core.datahub import DataHub
from grutopia.core.runtime.distributed_task_runtime_manager import (
    DistributedTaskRuntimeManager,
)
from grutopia.core.runtime.local_task_runtime_manager import LocalTaskRuntimeManager
from grutopia.core.runtime.task_runtime import (
    BaseTaskRuntimeManager,
    TaskRuntime,
    create_task_runtime_manager,
)
from grutopia.core.util import log


class SimulatorRuntime:
    """SimulatorRuntime"""

    def __init__(
        self,
        config_path: str = None,
        headless: bool = True,
        webrtc: bool = False,
        native: bool = False,
        config_class: Config = None,
    ):
        """

        Args:
            config_path: config file (yaml) path
        """
        self.env_num = 1
        self.config_file_path = config_path
        self.config = None
        self.simulator: Optional[SimConfig] = None
        self.task_runtime_manager: Optional[BaseTaskRuntimeManager] = None
        if self.config_file_path:
            self.init(config_dict=self.get_config_from_file())
        elif config_class:
            self.init(config_class=config_class)
        else:
            raise RuntimeError('No valid config is set.')

        # Init Isaac Sim
        from isaacsim import SimulationApp  # noqa

        self.headless = headless
        self._simulation_app = SimulationApp(
            {'headless': self.headless, 'anti_aliasing': 0, 'hide_ui': False, 'multi_gpu': False}
        )
        self._simulation_app._carb_settings.set('/physics/cooking/ujitsoCollisionCooking', False)
        log.debug('SimulationApp init done')

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
            if not file_path.endswith('.yaml') or file_path.endswith('.yml'):
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

    def init(self, config_dict: Dict = None, config_class: Config = None) -> None:
        if config_dict:
            config = Config(**config_dict)
        else:
            config = config_class
            self.config = config_class.model_dump()

        # Load Episodes file if episodes in task_config is instance of str.
        if isinstance(config.task_config.episodes, str):
            episodes_dict = self.read_yaml_file(config.task_config.episodes)
            config.task_config.episodes = EpisodeConfigFile(**episodes_dict).episodes

        # TO DELETE: Multiple episodes are not supported yet. Coming soon.
        if len(config.task_config.episodes) > 1:
            raise ValueError('multiple episodes are not supported yet !')

        # Init Datahub
        DataHub.datahub_init()

        _trm = create_task_runtime_manager(config.task_config)
        self.simulator = config.simulator
        self.task_runtime_manager = _trm
        self.env_num = _trm.env_num
        log.debug('SimulatorRuntime init done')

    def active_runtime(self) -> Dict[str, TaskRuntime]:
        """
        Get active runtimes.
        """
        return self.task_runtime_manager.active_runtime()
