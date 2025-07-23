from internutopia.core.config import Config, SimConfig
from internutopia.core.gym_env import Env
from internutopia.core.util import has_display
from internutopia.macros import gm
from internutopia_extension import import_extensions
from internutopia_extension.configs.robots.g1 import G1RobotCfg, move_by_speed_cfg
from internutopia_extension.configs.tasks import SingleInferenceTaskCfg
from internutopia_extension.interactions.keyboard import KeyboardInteraction

headless = False

if not has_display():
    headless = True


config = Config(
    simulator=SimConfig(physics_dt=1 / 240, rendering_dt=1 / 240, use_fabric=False, headless=headless, webrtc=headless),
    task_configs=[
        SingleInferenceTaskCfg(
            scene_asset_path=gm.ASSET_PATH + '/scenes/empty.usd',
            robots=[
                G1RobotCfg(
                    position=(0.0, 0.0, 0.8),
                    controllers=[move_by_speed_cfg],
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
    move_by_speed_cfg.name: (0.0, 0.0, 0.0),
}

print(f'actions: {env_action}')

keyboard = KeyboardInteraction()
while env.simulation_app.is_running():
    i += 1
    command = keyboard.get_input()
    x_speed = command[0] - command[1]
    y_speed = command[2] - command[3]
    z_speed = command[4] - command[5]
    env_action = {
        move_by_speed_cfg.name: (x_speed, y_speed, z_speed),
    }
    obs, _, terminated, _, _ = env.step(action=env_action)

    if i % 1000 == 0:
        print(i)

env.close()
