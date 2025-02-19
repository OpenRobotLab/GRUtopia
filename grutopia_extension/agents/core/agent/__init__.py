from grutopia_extension.agents.core.agent.agent import BaseAgent, create_agent

if BaseAgent.loop is None:
    BaseAgent.agent_loop_start()
