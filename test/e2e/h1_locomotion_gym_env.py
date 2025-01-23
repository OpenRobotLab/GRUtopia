from grutopia.core.gym_env import Env
from grutopia.core.runtime import SimulatorRuntime
from grutopia.core.util.agent import create_agent
from grutopia_extension import import_extensions

# Run with dummy agent
file_path = './GRUtopia/demo/configs/h1_locomotion_gym_env.yaml'

headless = True
webrtc = True

sim_runtime = SimulatorRuntime(config_path=file_path, headless=headless, webrtc=webrtc, native=True)

if not sim_runtime.agents:
    print(
        'No agents configured. Please review your configuration file and ensure that one agent is defined to proceed with the demo.'
    )

import_extensions()
# import custom extensions here.

env = Env(sim_runtime)
obs, info = env.reset()
print(f'========INIT OBS{obs}=============')

# init agent
agent = create_agent(sim_runtime=sim_runtime, reset_info=info)
print('========INIT AGENTS=============')
print(f'agent robot_name: {agent.robot_name}')

path = [(1.0, 0.0, 0.0), (1.0, 1.0, 0.0), (3.0, 4.0, 0.0)]
move_action = {'move_along_path': [path]}

i = 0
while env.simulation_app.is_running() and not env.finished():
    i += 1
    # fake action only for demo
    agent_action = agent.step(obs)
    agent_action = move_action

    # step
    obs, reward, terminated, truncated, info = env.step(action=agent_action)

    # reset env
    if terminated or truncated:
        obs, info = env.reset()
        agent = create_agent(sim_runtime=sim_runtime, reset_info=info)

    if i % 3 == 0:
        print(i)

env.close()
