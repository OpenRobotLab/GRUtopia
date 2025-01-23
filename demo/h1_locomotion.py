from grutopia.core.config import Config, EpisodeConfig, SimConfig, TaskConfig
from grutopia.core.gym_env import Env
from grutopia.core.runtime import SimulatorRuntime
from grutopia_extension import import_extensions
from grutopia_extension.config.robots.humanoid import (  # humanoid_camera,; humanoid_tp_camera,
    HumanoidRobot,
    humanoid_move_by_speed_controller,
)

h1_1 = HumanoidRobot(
    controllers=[
        humanoid_move_by_speed_controller,
    ],
    sensors=[
        # humanoid_camera.model_copy(
        #     update={'name': 'camera', 'size': (320, 240), 'enable': True}, deep=True
        # )
    ],
)

config = Config(
    simulator=SimConfig(physics_dt=1 / 240, rendering_dt=1 / 240, use_fabric=False),
    task_config=TaskConfig(
        type='SingleInferenceTask',
        task_name_prefix='h1_locomotion',
        episodes=[
            EpisodeConfig(
                scene_asset_path='GRUtopia/assets/scenes/empty.usd',
                scene_scale=[0.01, 0.01, 0.01],
                robots=[h1_1],
            ),
        ],
    ),
)

print(config.model_dump_json(indent=4))

sim_runtime = SimulatorRuntime(config_class=config, headless=True, webrtc=False, native=True)

import_extensions()
# import custom extensions here.

env = Env(sim_runtime)
obs, _ = env.reset()
# print(f'========INIT OBS{obs}=============')

import numpy as np
from omni.isaac.core.utils.rotations import euler_angles_to_quat

path = [(1.0, 0.0, 0.0), (1.0, 1.0, 0.0), (3.0, 4.0, 0.0)]
i = 0

# move_action = {'move_along_path': [path]}
move_action = {humanoid_move_by_speed_controller.name: [1.0, 0, 0]}
rotate_action = {'rotate': [euler_angles_to_quat(np.array([0, 0, np.pi]))]}
recover_action = {'recover': []}
keyboard_action = {'mh1_locomotion.pyove_with_keyboard': []}

while env.simulation_app.is_running():
    i += 1
    env_action = move_action
    obs, _, terminated, _, _ = env.step(action=env_action)
    if i % 500 == 0:
        print(i)
        print(obs)

    if (i - 100) % 2000 == 0:  # recover for 100 steps
        env_action = keyboard_action

    if 1 % 3000 == 0:
        env.reset()

env.simulation_app.close()
