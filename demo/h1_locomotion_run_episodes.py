from grutopia.core.gym_env import Env
from grutopia.core.runtime import SimulatorRuntime
from grutopia.core.util.container import is_in_container
from grutopia_extension import import_extensions

# # Run with single inference task
file_path = './GRUtopia/demo/configs/h1_locomotion_run_episodes.yaml'

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
# print(f'========INIT OBS{obs}=============')

path = [(1.0, 0.0, 0.0), (1.0, 1.0, 0.0), (3.0, 4.0, 0.0)]
i = 0

move_action = {'move_along_path': [path]}
recover_action = {'recover': []}

while env.simulation_app.is_running():
    i += 1
    env_action = move_action
    obs, _, terminated, _, _ = env.step(action=env_action)
    if i % 500 == 0:
        print(i)
        # print(obs)

    if i % 800 == 0:
        env_action = recover_action
        env.reset()

env.simulation_app.close()
