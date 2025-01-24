from grutopia.core.config import Config, SimConfig
from grutopia.core.gym_env import Env
from grutopia.core.runtime import SimulatorRuntime
from grutopia.core.util.container import is_in_container
from grutopia_extension import import_extensions
from grutopia_extension.configs.robots.g1 import G1RobotCfg, move_to_point_cfg
from grutopia_extension.configs.tasks import (
    SingleInferenceEpisodeCfg,
    SingleInferenceTaskCfg,
)

headless = False
webrtc = False

if is_in_container():
    headless = True
    webrtc = True


config = Config(
    simulator=SimConfig(physics_dt=1 / 240, rendering_dt=1 / 240, use_fabric=False),
    task_config=SingleInferenceTaskCfg(
        episodes=[
            SingleInferenceEpisodeCfg(
                scene_asset_path='GRUtopia/assets/scenes/empty.usd',
                robots=[
                    G1RobotCfg(
                        position=[0.0, 0.0, 0.8],
                        controllers=[move_to_point_cfg],
                    )
                ],
            ),
        ],
    ),
)

sim_runtime = SimulatorRuntime(config_class=config, headless=headless, webrtc=webrtc)

import_extensions()
# import custom extensions here.

env = Env(sim_runtime)
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

env.simulation_app.close()
