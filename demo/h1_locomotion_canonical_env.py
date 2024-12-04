from typing import Dict, List

from grutopia.core.agent import BaseAgent, create_agent
from grutopia.core.env import Env
from grutopia.core.runtime import SimulatorRuntime
from grutopia_extension import import_extensions

# Run with dummy agent
file_path = './GRUtopia/demo/configs/h1_locomotion_canonical_env.yaml'

headless = True
webrtc = True

sim_runtime = SimulatorRuntime(config_path=file_path, headless=headless, webrtc=webrtc, native=True)

import_extensions()
# import custom extensions here.

env = Env(sim_runtime)
obs, init_task_runtime_list = env.vector_reset()
print(f'========INIT OBS{obs}=============')

# init agent
task_name_to_agents_map: Dict[str, List[BaseAgent]] = {}
if sim_runtime.agents:
    for task_runtime in init_task_runtime_list:
        for agent_config in sim_runtime.agents:
            if task_runtime.name not in task_name_to_agents_map:
                task_name_to_agents_map[task_runtime.name] = []
            task_name_to_agents_map[task_runtime.name].append(
                create_agent(config=agent_config, task_name=task_runtime.name, task_runtime=task_runtime))

print('========INIT AGENTS=============')
for task_name, agents in task_name_to_agents_map.items():
    for agent in agents:
        print(f'task_name: {task_name} agent robot_name: {agent.robot_name}')

path = [(1.0, 0.0, 0.0), (1.0, 1.0, 0.0), (3.0, 4.0, 0.0)]
move_action = {'move_along_path': [path]}

i = 0
while env.simulation_app.is_running() and not env.finished():
    i += 1
    # get the actions
    env_actions = {}
    for task_name, task_obs in obs.items():
        task_action = {}
        for agent in task_name_to_agents_map[task_name]:
            agent_action = agent.step(task_obs)
            # fake action only for demo
            agent_action = move_action
            task_action[agent.robot_name] = agent_action
        env_actions[task_name] = task_action

    # step
    obs, terminated_status = env.step(actions=env_actions)

    # reset env if there is a terminated task
    for task_name, terminated in terminated_status.items():
        if terminated:
            obs, new_task_runtime = env.reset(task_name)
            del task_name_to_agents_map[task_name]

            # create new agents for a new task
            if new_task_runtime is not None:
                print(f'switch to a new task: {new_task_runtime.name} from previous task: {task_name}')
                task_name_to_agents_map[new_task_runtime.name] = []
                for agent_config in sim_runtime.agents:
                    task_name_to_agents_map[new_task_runtime.name].append(
                        create_agent(config=agent_config,
                                     task_name=new_task_runtime.name,
                                     task_runtime=new_task_runtime))

    if i % 10 == 0:
        print(i)

env.close()
