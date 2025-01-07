from grutopia.core.gym_env import Env
from grutopia.core.runtime import SimulatorRuntime
from grutopia.core.util.container import is_in_container
from grutopia_extension import import_extensions

# # Run with single inference task
file_path = './GRUtopia/demo/configs/h1_locomotion_loop.yaml'

headless = False
webrtc = False

if is_in_container():
    headless = True
    webrtc = True

sim_runtime = SimulatorRuntime(config_path=file_path, headless=headless, webrtc=False, native=True)

import_extensions()
# import custom extensions here.

env = Env(sim_runtime)
obs, _ = env.reset()

import numpy as np
from omni.isaac.core.utils.rotations import euler_angles_to_quat

path = [(1.0, 0.0, 0.0), (1.0, 1.0, 0.0), (3.0, 4.0, 0.0)]
i = 0

move_action = {'move_along_path': [path]}
rotate_action = {'rotate': [euler_angles_to_quat(np.array([0, 0, np.pi]))]}
recover_action = {'recover': []}
keyboard_action = {'mh1_locomotion.pyove_with_keyboard': []}

while env.simulation_app.is_running():
    i += 1
    env_action = move_action
    obs, _, _, _, _ = env.step(action=env_action)
    if i % 1000 == 0:
        print(i)
        print(obs)

    if i % 1500 == 0:
        env_action = recover_action
        env.reset()
        i = 1
    if (i - 100) % 3000 == 0:  # recover for 100 steps
        env_action = keyboard_action

env.simulation_app.close()
