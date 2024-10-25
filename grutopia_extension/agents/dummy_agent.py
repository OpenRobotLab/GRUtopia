import random
import time
from typing import Dict

from grutopia.core.agent import BaseAgent
from grutopia.core.runtime.task_runtime import TaskRuntime
from grutopia.core.util import log


@BaseAgent.register('DummyAgent')
class DummyAgent(BaseAgent):
    """
    Dummy Agent that does nothing. And set is_done at the 2nd call.
    """

    def __init__(self, task_name: str, robot_name: str | None, agent_config: Dict, sync_mode: str,
                 task_runtime: TaskRuntime):
        super().__init__(task_name, robot_name, agent_config, sync_mode, task_runtime)
        self.counter = 0
        log.debug(f'=============== agent_config: {agent_config} ===============')

    def decision_making(self):
        time.sleep(float(random.randint(3, 5)))
        self.counter += 1
        log.info(f'Task_id: {self.task_name}, DummyAgent.counter: {self.counter}')
        if self.counter % 2 == 0:
            log.info(self.counter)
            self.terminate()
