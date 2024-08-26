from grutopia.core.datahub import DataHub
from grutopia.core.env import Env
from grutopia.core.runtime import SimulatorRuntime
from grutopia_extension import import_extensions

file_path = './GRUtopia/demo/configs/h1_npc.yaml'
sim_runtime = SimulatorRuntime(config_path=file_path, headless=True, webrtc=True, native=False)

import_extensions()
# import custom extensions here.

env = Env(sim_runtime)

i = 0

actions = {'h1': {'move_with_keyboard': []}}

while env.simulation_app.is_running():
    i += 1
    env_actions = {}
    for task_runtime in env.active_runtimes.values():
        env_actions[task_runtime.name] = actions
    obs = env.step(actions=env_actions)

    if i % 5000 == 0:
        print(i)

DataHub.clear()
env.simulation_app.close()
