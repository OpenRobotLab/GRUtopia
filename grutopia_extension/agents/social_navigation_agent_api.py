from typing import Dict

from grutopia.core.agent import BaseAgent
from grutopia.core.util import log


@BaseAgent.register('SocialNavigationAgent')
class SocialNavigationAgent(BaseAgent):
    """
    Dummy Agent that does nothing. And set is_done at the 20th call.
    """

    def __init__(self, task_name: str, robot_name: str | None, agent_config: Dict, sync_mode: str):
        super().__init__(task_name, robot_name, agent_config, sync_mode)
        log.debug(f'=============== agent_config: {agent_config} ===============')

        # TODO Add runtime validate here.
        if 'xxx' not in agent_config:
            raise KeyError('xxx not in agent_config. Please check your runtime.')

    def decision_making(self):
        # TODO Add logic here.
        pass
