from grutopia.core.env import Env
from grutopia.core.runtime import SimulatorRuntime
from grutopia.core.util.container import is_in_container
from grutopia_extension import import_extensions

# # Run with single inference task
file_path = './GRUtopia/demo/configs/h1_locomotion.yaml'

headless = False
webrtc = False

if is_in_container():
    headless = True
    webrtc = True

sim_runtime = SimulatorRuntime(config_path=file_path, headless=headless, webrtc=webrtc)

import_extensions()
# import custom extensions here.

env = Env(sim_runtime)
obs, _ = env.vector_reset()
print(f'========INIT OBS{obs}=============')

import numpy as np
from omni.isaac.core.utils.rotations import euler_angles_to_quat

path = [(1.0, 0.0, 0.0), (1.0, 1.0, 0.0), (3.0, 4.0, 0.0)]
i = 0

move_action = {'move_along_path': [path]}
rotate_action = {'rotate': [euler_angles_to_quat(np.array([0, 0, np.pi]))]}
path_finished = False
recover_action = {'recover': []}
keyboard_action = {'move_with_keyboard': []}
actions = {'h1_0': move_action}

while env.simulation_app.is_running():
    i += 1
    env_actions = {}
    for task_runtime in env.active_runtimes.values():
        env_actions[task_runtime.name] = actions

    env.step(actions=env_actions)

    if i % 1000 == 0:
        print(i)

    if i % 3000 == 0:
        actions['h1_0'] = recover_action
    if (i - 100) % 3000 == 0:  # recover for 100 steps
        actions['h1_0'] = keyboard_action

env.simulation_app.close()
