from grutopia.core.config import Config, SimConfig
from grutopia.core.gym_env import Env
from grutopia.core.util import has_display
from grutopia.macros import gm
from grutopia_extension import import_extensions
from grutopia_extension.configs.metrics.simple_metric import SimpleMetricCfg
from grutopia_extension.configs.robots.h1 import (
    H1RobotCfg,
    h1_camera_cfg,
    h1_tp_camera_cfg,
    move_along_path_cfg,
    move_by_speed_cfg,
    rotate_cfg,
)
from grutopia_extension.configs.tasks import FiniteStepTaskCfg

headless = False
if not has_display():
    headless = True

h1_1 = H1RobotCfg(
    position=(0.0, 0.0, 1.05),
    controllers=[
        move_by_speed_cfg,
        move_along_path_cfg,
        rotate_cfg,
    ],
    sensors=[
        h1_camera_cfg.update(name='camera', resolution=(320, 240), enable=True),
        h1_tp_camera_cfg.update(enable=False),
    ],
)

config = Config(
    simulator=SimConfig(physics_dt=1 / 240, rendering_dt=1 / 240, use_fabric=False, headless=headless, native=headless),
    metrics_save_path='./h1_simple_metric.jsonl',
    task_configs=[
        FiniteStepTaskCfg(
            max_steps=300,
            metrics=[SimpleMetricCfg(metric_config={'robot_name': 'h1'})],
            scene_asset_path=gm.ASSET_PATH + '/scenes/empty.usd',
            scene_scale=(0.01, 0.01, 0.01),
            robots=[h1_1],
        ),
    ],
)

import_extensions()

env = Env(config)
obs, _ = env.reset()
print(f'========INIT OBS{obs}=============')

path = [(1.0, 0.0, 0.0), (1.0, 1.0, 0.0), (3.0, 4.0, 0.0)]
i = 0

move_action = {move_along_path_cfg.name: [path]}

while env.simulation_app.is_running():
    i += 1
    action = move_action
    obs, _, terminated, _, _ = env.step(action=action)

    if terminated:
        obs, info = env.reset()
        if env.RESET_INFO_TASK_CONFIG not in info:  # No more episode
            break

    if i % 100 == 0:
        print(i)

env.close()
