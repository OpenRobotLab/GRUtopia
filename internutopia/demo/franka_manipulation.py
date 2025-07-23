import numpy as np

from internutopia.core.config import Config, SimConfig
from internutopia.core.gym_env import Env
from internutopia.core.util import has_display
from internutopia.macros import gm
from internutopia_extension import import_extensions
from internutopia_extension.configs.metrics import RecordingMetricCfg
from internutopia_extension.configs.robots.franka import (
    FrankaRobotCfg,
    arm_ik_cfg,
    gripper_cfg,
)
from internutopia_extension.configs.tasks import ManipulationTaskCfg

headless = not has_display()

franka = FrankaRobotCfg(
    position=(0, 0, 0),
    controllers=[
        arm_ik_cfg,
        gripper_cfg,
    ],
)

config = Config(
    simulator=SimConfig(physics_dt=1 / 240, rendering_dt=1 / 240, use_fabric=False, headless=headless, webrtc=True),
    task_configs=[
        ManipulationTaskCfg(
            metrics=[
                RecordingMetricCfg(
                    robot_name='franka',
                    fields=['joint_action'],
                )
            ],
            scene_asset_path=gm.ASSET_PATH + '/scenes/empty.usd',
            robots=[franka],
            prompt='Prompt test 1',
            target='franka_manipulation',
            episode_idx=0,
            max_steps=2000,
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
while env.simulation_app.is_running():
    i += 1
    if i % 400 == 0:
        env_actions = actions[0]
    elif i % 400 == 100:
        env_actions = actions[1]
    elif i % 400 == 200:
        env_actions = actions[2]
    elif i % 400 == 300:
        env_actions = actions[3]
    else:
        env_actions = {}

    obs, _, terminated, _, _ = env.step(action=env_actions)

    if terminated:
        obs, info = env.reset()
        if info is None:  # No more episode
            break

    if i % 1000 == 0:
        print(i)

env.close()
