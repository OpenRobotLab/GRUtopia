from typing import List

from grutopia.core.gym_env import Env
from grutopia.core.runtime.task_runtime import TaskRuntime
from grutopia_extension.agents.config import AgentCfg
from grutopia_extension.agents.core.agent import BaseAgent
from grutopia_extension.agents.core.agent import create_agent as _create_agent


def create_agents(agent_cfgs: List[AgentCfg], reset_info: dict) -> list[BaseAgent]:
    """
    Creates a list of agent clients based on the provided configurations and reset information.

    Args:
        agent_cfgs: A list of configurations for each agent.
        reset_info: A dictionary containing reset information, which may include task runtime details.

    Returns:
        A list of BaseAgent instances created from the provided configurations and reset information.
    """
    agents: List[BaseAgent] = []
    if Env.RESET_INFO_TASK_RUNTIME in reset_info:
        current_task: TaskRuntime = reset_info[Env.RESET_INFO_TASK_RUNTIME]
        for agent_cfg in agent_cfgs:
            agents.append(_create_agent(config=agent_cfg, task_name=current_task.name, task_runtime=current_task))

    return agents


def create_agent(agent_cfg: AgentCfg, reset_info: dict) -> BaseAgent:
    """
    Creates an agent based on the provided configuration and reset information.

    Args:
        agent_cfg (AgentCfg): Configuration for the agent.
        reset_info (dict): Information used to reset the environment, which may
            include task runtime details.

    Returns:
        BaseAgent: The created agent, or None if the necessary reset information is not present.
    """
    agent = None
    if Env.RESET_INFO_TASK_RUNTIME in reset_info:
        current_task: TaskRuntime = reset_info[Env.RESET_INFO_TASK_RUNTIME]
        agent = _create_agent(config=agent_cfg, task_name=current_task.name, task_runtime=current_task)
    return agent
