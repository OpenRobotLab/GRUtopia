from grutopia.core.datahub import DataHub
from grutopia.core.gym_env import Env
from grutopia.core.runtime import SimulatorRuntime
from grutopia_extension import import_extensions

file_path = './GRUtopia/demo/configs/h1_npc.yaml'
sim_runtime = SimulatorRuntime(config_path=file_path, headless=True, webrtc=True, native=False)

import_extensions()
# import custom extensions here.

env = Env(sim_runtime)
obs, _ = env.reset()
print(f'========INIT OBS{obs}=============')

keyboard_action = {'move_with_keyboard': []}
i = 0
while env.simulation_app.is_running():
    i += 1
    env.step(action=keyboard_action)

    if i % 5000 == 0:
        print(i)

DataHub.clear()
env.close()
