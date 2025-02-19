from enum import Enum
from typing import Any, Dict, Optional, Union

from pydantic import BaseModel


class AgentModeEnum(str, Enum):
    sync_mode = 'sync'
    async_mode = 'async'


class AgentCfg(BaseModel):
    """
    Class representing the configuration for an agent, including its type, robot association, synchronization mode, and additional settings.

    This class is designed to encapsulate the necessary parameters to configure an agent within a system. It inherits from `BaseModel`, indicating it's a data model class likely used within a framework that supports data validation and parsing, such as Pydantic.

    Attributes:
        type (str): Specifies the type of the agent. This could refer to the functional category or a specific implementation detail of the agent.

        robot_name (Optional[Union[str, None]], optional): The name of the robot associated with the agent. Currently marked as optional but is planned to become a required field in the future, potentially also facilitating non-physical agents (e.g., NPCs).

        sync_mode (Optional[AgentModeEnum], optional): Defines the synchronization mode for the agent, defaulting to `AgentModeEnum.sync_mode`. This enum determines how the agent operates concerning real-time updates or processing.

        agent_config (Dict[str, Any]): Additional configuration settings for the agent, provided as a dictionary where keys are strings and values can be of any type. This allows for flexible extension of agent-specific configurations without altering the base class structure.

    Note: Type hints are provided for parameters, hence types are not redundantly described in the docstring.
    """

    type: str
    # TODO: `robot_name` will be changed to a required parameter later.
    #       And consider adding a disembodied robot for NPCs.
    robot_name: Optional[Union[str, None]] = None
    sync_mode: Optional[AgentModeEnum] = AgentModeEnum.sync_mode
    agent_config: Optional[Dict[str, Any]] = {}
