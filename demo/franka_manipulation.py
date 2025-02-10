from grutopia.core.config import Config, SimConfig
from grutopia.core.gym_env import Env
from grutopia.core.runtime import SimulatorRuntime
from grutopia_extension import import_extensions
from grutopia_extension.configs.metrics import RecordingMetricCfg
from grutopia_extension.configs.robots.franka import (
    FrankaRobotCfg,
    arm_ik_cfg,
    gripper_cfg,
)
from grutopia_extension.configs.tasks import (
    ManipulationEpisodeCfg,
    ManipulationExtra,
    ManipulationTaskCfg,
    ManipulationTaskSetting,
)

franka = FrankaRobotCfg(
    position=[0, 0, 0],
    controllers=[
        arm_ik_cfg,
        gripper_cfg,
    ],
)

config = Config(
    simulator=SimConfig(physics_dt=1 / 240, rendering_dt=1 / 240, use_fabric=False),
    task_config=ManipulationTaskCfg(
        metrics=[
            RecordingMetricCfg(
                robot_name='franka',
                fields=['joint_action'],
            )
        ],
        episodes=[
            ManipulationEpisodeCfg(
                scene_asset_path='GRUtopia/assets/scenes/empty.usd',
                robots=[franka],
                extra=ManipulationExtra(
                    prompt='Prompt test 1',
                    target='franka_manipulation',
                    episode_idx=0,
                ),
            ),
        ],
        task_settings=ManipulationTaskSetting(max_step=2000),
    ),
)

sim_runtime = SimulatorRuntime(config_class=config, headless=True, webrtc=False, native=True)

import_extensions()
import numpy as np
from omni.isaac.core.utils.rotations import euler_angles_to_quat

env = Env(sim_runtime)
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

    obs, _, _, _, _ = env.step(action=env_actions)

    if i % 1000 == 0:
        print(i)

env.simulation_app.close()
