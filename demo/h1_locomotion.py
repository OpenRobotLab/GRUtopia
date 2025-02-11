from grutopia.core.config import Config, SimConfig
from grutopia.core.gym_env import Env
from grutopia.core.runtime import SimulatorRuntime
from grutopia.core.util import has_display
from grutopia_extension import import_extensions
from grutopia_extension.configs.robots.humanoid import (
    HumanoidRobotCfg,
    humanoid_camera_cfg,
    move_along_path_cfg,
    move_by_speed_cfg,
    rotate_cfg,
)
from grutopia_extension.configs.tasks import (
    SingleInferenceEpisodeCfg,
    SingleInferenceTaskCfg,
)

headless = False
if not has_display():
    headless = True

h1_1 = HumanoidRobotCfg(
    position=(0.0, 0.0, 1.05),
    controllers=[
        move_by_speed_cfg,
        move_along_path_cfg,
        rotate_cfg,
    ],
    sensors=[
        humanoid_camera_cfg.model_copy(update={'name': 'camera', 'resolution': (320, 240), 'enable': True}, deep=True)
    ],
)

config = Config(
    simulator=SimConfig(physics_dt=1 / 240, rendering_dt=1 / 240, use_fabric=False),
    task_config=SingleInferenceTaskCfg(
        episodes=[
            SingleInferenceEpisodeCfg(
                scene_asset_path='GRUtopia/assets/scenes/empty.usd',
                scene_scale=(0.01, 0.01, 0.01),
                robots=[h1_1],
            ),
        ],
    ),
)

print(config.model_dump_json(indent=4))

sim_runtime = SimulatorRuntime(config_class=config, headless=headless, native=headless)

import_extensions()
# import custom extensions here.

env = Env(sim_runtime)
obs, _ = env.reset()
# print(f'========INIT OBS{obs}=============')

path = [(1.0, 0.0, 0.0), (1.0, 1.0, 0.0), (3.0, 4.0, 0.0)]
i = 0

move_action = {move_along_path_cfg.name: [path]}

while env.simulation_app.is_running():
    i += 1
    env_action = move_action
    obs, _, terminated, _, _ = env.step(action=env_action)
    if i % 500 == 0:
        print(i)
        # print(obs)

env.simulation_app.close()
