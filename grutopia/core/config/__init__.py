from copy import deepcopy
from typing import List, Optional, Union

import numpy as np
import yaml
from pydantic import BaseModel

from grutopia.core.config.npc import NPCUserConfig
from grutopia.core.config.robot import RobotUserConfig
from grutopia.core.config.scene import Object, Scene
from grutopia.core.config.task import TaskUserConfig
from grutopia.core.util import log


class Env(BaseModel):
    """
    Env config
    """
    # background config(type None for nothing)
    bg_type: Union[str, None] = None
    bg_path: Optional[str]


class SimConfig(BaseModel):
    """
    SimConfig
    """
    physics_dt: Optional[float | str] = None
    rendering_dt: Optional[float | str] = None
    rendering_interval: Optional[int] = None


class Config(BaseModel):
    """
    Config
    """
    simulator: Optional[SimConfig]
    env_set: Optional[Env] = None
    tasks: List[TaskUserConfig]
    npc: List[NPCUserConfig] = []


class ConfigDictValidator(BaseModel):
    simulator: Optional[SimConfig]
    env_set: Optional[Env] = None
    task_config: List[TaskUserConfig]


class SimulatorConfig:

    def __init__(self, path: str = None):
        """

        Args:
            path: config file path
        """
        self.env_num = 1
        self.offset_size = None
        self.config_file_path = path
        self.config_dict = None
        self.config: Config = self.validate(self.get_config_from_file())

    def get_config_from_file(self):
        if self.config_file_path:
            if not self.config_file_path.endswith('.yaml') or \
                    self.config_file_path.endswith('.yml'):
                log.error('config file not end with .yaml or .yml')
                raise FileNotFoundError('config file not end with .yaml or .yml')
            with open(self.config_file_path, 'r') as f:
                self.config_dict = yaml.load(f.read(), yaml.FullLoader)
            return self.config_dict
        log.error('Config file path is not set')
        raise FileNotFoundError('Config file path is not set')

    def validate(self, config_dict) -> Config:
        _env = None
        _render = None
        self.env_num = 0
        if 'tasks' not in config_dict:
            raise KeyError('tasks are not set in config path')

        if 'render' in config_dict:
            _render = config_dict['render']
        else:
            _render = {'render': True}

        if 'env' in config_dict:
            _env = config_dict['env']
        else:
            _env = {'bg_type': 'default'}

        if 'npc' in config_dict:
            _npc = config_dict['npc']
        else:
            _npc = []

        for _task in config_dict['tasks']:
            self.env_num += _task['env_num']
            if 'offset_size' in _task and isinstance(_task['offset_size'], float):
                if self.offset_size is None:
                    self.offset_size = _task['offset_size']
                else:
                    self.offset_size = max(self.offset_size, _task['offset_size'])
        _column_length = int(np.sqrt(self.env_num))
        if self.offset_size is None:
            # default
            self.offset_size = 10.0

        env_count = 0
        tasks = []
        for _task in config_dict['tasks']:
            for i in range(_task['env_num']):
                task_copy = deepcopy(_task)

                row = int(i // _column_length)
                column = i % _column_length
                offset = [row * self.offset_size, column * self.offset_size, 0]

                task_copy['name'] = f"{task_copy['name']}_{env_count}"

                task_copy['root_path'] = f'/World/env_{env_count}'
                task_copy['env_id'] = env_count

                task_copy['offset'] = offset
                if 'scene_root_path' not in _task:
                    task_copy['scene_root_path'] = '/scene'
                if 'robots_root_path' not in _task:
                    task_copy['robots_root_path'] = '/robots'
                if 'objects_root_path' not in _task:
                    task_copy['objects_root_path'] = '/objects'

                for r in task_copy['robots']:
                    r['name'] = f"{r['name']}_{env_count}"
                    r['prim_path'] = task_copy['root_path'] + task_copy['robots_root_path'] + r['prim_path']
                    r['position'] = [task_copy['offset'][idx] + i for idx, i in enumerate(r['position'])]
                if 'objects' in task_copy and task_copy['objects'] is not None:
                    for o in task_copy['objects']:
                        o['name'] = f"{o['name']}_{env_count}"
                        o['prim_path'] = task_copy['root_path'] + task_copy['objects_root_path'] + o['prim_path']
                        o['position'] = [task_copy['offset'][idx] + i for idx, i in enumerate(o['position'])]
                tasks.append(TaskUserConfig(**task_copy))
                env_count += 1

        # log.debug(tasks)
        return Config(simulator=SimConfig(**config_dict['simulator']),
                      render=_render,
                      env_set=_env,
                      tasks=tasks,
                      npc=_npc)


class SimulatorConfigWithDataset(SimulatorConfig):

    def __init__(self, path: str = None):
        super().__init__(path)

    def validate(self, config_dict) -> Config:
        with open(config_dict['dataset_path'], 'r') as f:
            # TODO finish this implementation
            dataset: List = yaml.load(f.read(), yaml.FullLoader)
        return super().validate(config_dict)
