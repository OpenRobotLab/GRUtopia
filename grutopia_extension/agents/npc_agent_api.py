from typing import Dict

from grutopia.core.agent import BaseAgent
from grutopia.core.util import log
from grutopia_extension.agents.npc_agent import NPC
# Load NPC form NPC repo
from grutopia_extension.agents.npc_agent.config import NPCUserConfig


@BaseAgent.register('NPCAgent')
class NPCAgent(BaseAgent):
    """
    NPC Agent.

    This agent won't terminated.
    """

    def __init__(self, task_name: str, robot_name: str | None, agent_config: Dict, sync_mode: str):
        super().__init__(task_name, robot_name, agent_config, sync_mode)
        log.debug(f'=============== agent_config: {agent_config} ===============')
        try:
            cfg = NPCUserConfig(**agent_config)
        except Exception as e:
            log.error('agent_config of this agent(NPC) is not valid (By grutopia.core.runtime.npc.NPCUserConfig)')
            raise e
        self.npc = NPC(cfg)

    def decision_making(self):
        """
        This agent won't be terminated.
        """
        self.npc.feed(self.get_observation())
