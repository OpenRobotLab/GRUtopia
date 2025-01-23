from grutopia.core.gym_env import Env
from grutopia.core.runtime import SimulatorRuntime
from grutopia_extension import import_extensions

file_path = './GRUtopia/demo/configs/franka_manipulation_mocap_teleop.yaml'
sim_runtime = SimulatorRuntime(config_path=file_path, headless=False, webrtc=False, native=True)

import_extensions()
from grutopia_extension.interactions.motion_capture import MocapInteraction

env = Env(sim_runtime)
obs, _ = env.reset()

mocap_url = 'http://127.0.0.1:5001'
mocap_interaction = MocapInteraction(mocap_url)

while env.simulation_app.is_running():
    cur_mocap_info = mocap_interaction.step()
    arm_action = {'franka_mocap_teleop_controller': [cur_mocap_info]}

    obs, _, _, _, _ = env.step(action=arm_action)

mocap_interaction.server_stop()
env.simulation_app.close()
