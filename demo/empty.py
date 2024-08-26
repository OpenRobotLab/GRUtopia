from grutopia.core.env import Env
from grutopia.core.runtime import SimulatorRuntime
from grutopia_extension import import_extensions

file_path = './GRUtopia/demo/configs/empty.yaml'

sim_runtime = SimulatorRuntime(config_path=file_path, headless=True, webrtc=False, native=True)

import_extensions()
# import custom extensions here.

env = Env(sim_runtime)

while env.simulation_app.is_running():
    obs = env.step(actions={})

env.simulation_app.close()
