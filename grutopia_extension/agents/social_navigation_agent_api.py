from typing import Dict

import yaml

from grutopia.core.agent import BaseAgent
from grutopia.core.datahub.datahub import ActionData
from grutopia.core.runtime.task_runtime import TaskRuntime
from grutopia.core.util import log


@BaseAgent.register('SocialNavigationAgent')
class SocialNavigationAgent(BaseAgent):
    """
    Dummy Agent that does nothing. And set is_done at the 20th call.
    """

    def __init__(
        self, task_name: str, robot_name: str | None, agent_config: Dict, sync_mode: str, task_runtime: TaskRuntime
    ):
        super().__init__(task_name, robot_name, agent_config, sync_mode, task_runtime)
        log.debug(f'=============== agent_config: {agent_config} ===============')

        # # TODO Add runtime validate here.
        # if 'robot_name' not in agent_config:
        #     raise KeyError('xxx not in agent_config. Please check your runtime.')
        from grutopia_extension.agents.social_navigation_agent.social_navigation_agent import (
            SocialNavigationAgent,
        )

        with open('./GRUtopia/grutopia_extension/agents/social_navigation_agent/agent_config.yaml', 'r') as file:
            agent_config = yaml.safe_load(file)
        self.agent = SocialNavigationAgent(agent_config)
        self.agent.reset(self.task_runtime.extra['question'])

    def decision_making(self, obs):
        # obs = self.get_observation().copy()
        if obs['sim_step'] > self.task_runtime.task_settings['max_step']:
            self.terminate()
        chat_control = self.chat.get_message()
        if len(chat_control) > 0 and chat_control[0]['name'] == 'npc' + '_' + str(self.task_runtime.env.env_id):
            obs['answer'] = chat_control[0]['message']
            self.set_observation(obs)
        render = obs.pop('render')
        agent_return = self.agent.act(obs, render=render)
        # agent_return = {'finish': {}}
        controllers = []
        while len(agent_return) > 0:
            controller, data = agent_return.popitem()
            if controller == 'finish':
                if render:
                    self.terminate()
                continue
            if controller == 'web_chat':
                at = 'npc' + '_' + str(self.task_runtime.env.env_id)
                data = 'The goal object is ' + self.task_runtime.extra['target'] + '. ' + data
                self.chat.send_message(data, [at], 0)
                obs['question'] = data
                self.set_observation(obs)
                continue
            controllers.append({'controller': controller, 'data': data})
        action = ActionData(**{'robot': self.robot_name, 'controllers': controllers})
        self._set_actions({self.task_name: action})
