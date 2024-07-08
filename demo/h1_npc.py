from grutopia.core.config import SimulatorConfig
from grutopia.core.datahub.web_ui_api import clear as webui_clear
from grutopia.core.env import BaseEnv

file_path = './GRUtopia/demo/configs/h1_npc.yaml'
sim_config = SimulatorConfig(file_path)

env = BaseEnv(sim_config, headless=True, webrtc=True)

task_name = env.config.tasks[0].name
robot_name = env.config.tasks[0].robots[0].name

i = 0

actions = {'h1': {'move_with_keyboard': []}}

while env.simulation_app.is_running():
    i += 1
    env_actions = []
    env_actions.append(actions)
    obs = env.step(actions=env_actions)

    if i % 100 == 0:
        print(i)

webui_clear()
env.simulation_app.close()
