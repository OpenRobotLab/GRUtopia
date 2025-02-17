from typing import Optional

from grutopia.core.agent import BaseAgent
from grutopia.core.agent import create_agent as _create_agent
from grutopia.core.gym_env import Env
from grutopia.core.runtime import SimulatorRuntime
from grutopia.core.runtime.task_runtime import TaskRuntime


def create_agent(sim_runtime: SimulatorRuntime, reset_info: dict) -> Optional[BaseAgent]:
    """
    Creates an agent based on the given simulator runtime and environment reset information.

    Args:
        sim_runtime (SimulatorRuntime): The simulator runtime containing the agent configuration.
        reset_info (dict): The information returned by the environment reset.

    Returns:
        Optional[BaseAgent]: An instance of the agent if the reset information contains the task runtime, otherwise None.
    """
    agent: BaseAgent = None
    current_task = None
    if Env.RESET_INFO_TASK_RUNTIME in reset_info:
        current_task: TaskRuntime = reset_info[Env.RESET_INFO_TASK_RUNTIME]
        agent = _create_agent(config=sim_runtime.agents[0], task_name=current_task.name, task_runtime=current_task)

    return agent
