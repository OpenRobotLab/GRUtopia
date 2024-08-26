from grutopia.core.env import Env
from grutopia.core.runtime import SimulatorRuntime
from grutopia.core.util.container import is_in_container
from grutopia_extension import import_extensions

# # Run with single inference task
# file_path = './GRUtopia/demo/configs/h1_locomotion.yaml'

# Run with dummy agent
file_path = './GRUtopia/demo/configs/h1_locomotion_agent.yaml'

# # Run with dummy agent. And episodes configured with file path.
# file_path = './GRUtopia/demo/configs/h1_locomotion_agent_read_episodes_file.yaml'

sim_runtime = SimulatorRuntime(config_path=file_path, headless=True, webrtc=False, native=True)

import_extensions()
# import custom extensions here.

headless = False
webrtc = False

if is_in_container():
    headless = True
    webrtc = True

env = Env(sim_runtime)

import numpy as np
from omni.isaac.core.utils.rotations import euler_angles_to_quat

path = [(1.0, 0.0, 0.0), (1.0, 1.0, 0.0), (3.0, 4.0, 0.0)]
i = 0

move_action = {'move_along_path': [path]}
rotate_action = {'rotate': [euler_angles_to_quat(np.array([0, 0, np.pi]))]}
path_finished = False
actions = {'h1': move_action}

while env.simulation_app.is_running():
    i += 1
    env_actions = {}
    for task_runtime in env.active_runtimes.values():
        env_actions[task_runtime.name] = actions

    obs = env.step(actions=env_actions)

    if i % 1000 == 0:
        print(i)

env.simulation_app.close()
