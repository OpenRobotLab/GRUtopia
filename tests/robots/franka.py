import numpy as np

from grutopia.core.config import Config, SimConfig
from grutopia.core.gym_env import Env
from grutopia.core.util import has_display
from grutopia.macros import gm
from grutopia_extension import import_extensions
from grutopia_extension.configs.robots.franka import (
    FrankaRobotCfg,
    arm_ik_cfg,
    gripper_cfg,
)
from grutopia_extension.configs.tasks import SingleInferenceTaskCfg

headless = not has_display()

franka = FrankaRobotCfg(
    position=(0, 0, 0),
    controllers=[
        arm_ik_cfg,
        gripper_cfg,
    ],
)

config = Config(
    simulator=SimConfig(physics_dt=1 / 240, rendering_dt=1 / 240, use_fabric=True, headless=headless, native=headless),
    task_configs=[
        SingleInferenceTaskCfg(
            scene_asset_path=gm.ASSET_PATH + '/scenes/empty.usd',
            robots=[franka],
        ),
    ],
)

import_extensions()

env = Env(config)
from omni.isaac.core.utils.rotations import euler_angles_to_quat

obs, _ = env.reset()
print(f'========INIT OBS{obs}=============')

actions = [
    {arm_ik_cfg.name: [np.array([0.4, 0, 0.45]), euler_angles_to_quat((0.0, 0.0, 0.0))]},
    {gripper_cfg.name: ['open']},
    {arm_ik_cfg.name: [np.array([0.4, 0.4, 0.1]), euler_angles_to_quat((np.pi / 2, np.pi / 2, np.pi / 2))]},
    {gripper_cfg.name: ['close']},
]

i = 0
action_idx = 0
max_steps = 2000
while env.simulation_app.is_running():
    i += 1
    env_action = actions[action_idx]

    obs, _, terminated, _, _ = env.step(action=env_action)
    finished = obs['controllers'][arm_ik_cfg.name]['finished']
    if finished:
        print(f'action finished: env_action={env_action}, obs={obs}')
        action_idx += 1
        if action_idx >= len(actions):
            print('all actions finished')
            break

    assert i < max_steps, f'max steps reached, env_action={env_action}, obs={obs}'
    if i % 1000 == 0:
        print(i)
        print(obs)


env.close()
