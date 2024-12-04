from grutopia.core.env import Env
from grutopia.core.runtime import SimulatorRuntime
from grutopia_extension import import_extensions

# Run with dummy agent
file_path = './GRUtopia/demo/configs/h1_social_navigation.yaml'

sim_runtime = SimulatorRuntime(config_path=file_path, headless=True, webrtc=False, native=True)

import_extensions()
# import custom extensions here.

env = Env(sim_runtime)
env.vector_reset()

import numpy as np
from omni.isaac.core.utils.rotations import euler_angles_to_quat

path = [(1.0, 0.0, 0.0), (1.0, 1.0, 0.0), (3.0, 4.0, 0.0)]
i = 0

move_action = {'move_along_path': [path]}
rotate_action = {'rotate': [euler_angles_to_quat(np.array([0, 0, np.pi]))]}
path_finished = False
actions = {'h1_0': move_action}

while env.simulation_app.is_running() and not env.finished():
    i += 1
    env_actions = {}
    for task_runtime in sim_runtime.active_runtime().values():
        env_actions[task_runtime.name] = actions

    _, terminated_status = env.step(actions=env_actions)

    for task_name, terminated in terminated_status.items():
        if terminated:
            env.reset(task_name)

    if i % 1000 == 0:
        print(i)

env.close()
