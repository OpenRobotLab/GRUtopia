from enum import Enum
from typing import Any, Dict, Optional, Union

from pydantic import BaseModel


class AgentModeEnum(str, Enum):
    sync_mode = 'sync'
    async_mode = 'async'


class AgentConfig(BaseModel):
    type: str
    # TODO: `robot_name` will be changed to a required parameter later.
    #       And consider adding a disembodied robot for NPCs.
    robot_name: Optional[Union[str, None]] = None
    sync_mode: Optional[AgentModeEnum] = AgentModeEnum.sync_mode
    agent_config: Dict[str, Any] = {}
