from grutopia.core.gym_env import Env
from grutopia.core.runtime import SimulatorRuntime
from grutopia_extension import import_extensions

file_path = './GRUtopia/demo/configs/franka_manipulation.yaml'

sim_runtime = SimulatorRuntime(config_path=file_path, headless=False, webrtc=False, native=True)

import_extensions()
import numpy as np
from omni.isaac.core.utils.rotations import euler_angles_to_quat

env = Env(sim_runtime)
obs, _ = env.reset()
print(f'========INIT OBS{obs}=============')

actions = [
    {'arm_ik_controller': [np.array([0.4, 0, 0.45]), euler_angles_to_quat((0.0, 0.0, 0.0))]},
    {'gripper_controller': ['open']},
    {'arm_ik_controller': [np.array([0.4, 0.4, 0.1]), euler_angles_to_quat((np.pi / 2, np.pi / 2, np.pi / 2))]},
    {'gripper_controller': ['close']},
]

i = 0
while env.simulation_app.is_running():
    i += 1
    if i % 400 == 0:
        env_actions = actions[0]
    elif i % 400 == 100:
        env_actions = actions[1]
    elif i % 400 == 200:
        env_actions = actions[2]
    elif i % 400 == 300:
        env_actions = actions[3]
    else:
        env_actions = {}

    obs, _, _, _, _ = env.step(action=env_actions)

    if i % 1000 == 0:
        print(i)

env.simulation_app.close()
