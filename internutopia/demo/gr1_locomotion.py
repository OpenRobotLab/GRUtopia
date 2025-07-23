from internutopia.core.config import Config, SimConfig
from internutopia.core.gym_env import Env
from internutopia.core.util import has_display
from internutopia.macros import gm
from internutopia_extension import import_extensions
from internutopia_extension.configs.robots.gr1 import GR1RobotCfg, move_to_point_cfg
from internutopia_extension.configs.tasks import SingleInferenceTaskCfg

headless = False

if not has_display():
    headless = True

config = Config(
    simulator=SimConfig(physics_dt=1 / 240, rendering_dt=1 / 240, use_fabric=False, headless=headless, webrtc=headless),
    task_configs=[
        SingleInferenceTaskCfg(
            scene_asset_path=gm.ASSET_PATH + '/scenes/empty.usd',
            robots=[
                GR1RobotCfg(
                    position=(0.0, 0.0, 0.95),
                    controllers=[move_to_point_cfg],
                )
            ],
        ),
    ],
)

import_extensions()

env = Env(config)
obs, _ = env.reset()

i = 0

env_action = {
    move_to_point_cfg.name: [(3.0, 3.0, 0.0)],
}

print(f'actions: {env_action}')

while env.simulation_app.is_running():
    i += 1
    obs, _, terminated, _, _ = env.step(action=env_action)

    if i % 1000 == 0:
        print(i)

env.close()
