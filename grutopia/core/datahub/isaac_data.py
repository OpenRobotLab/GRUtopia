from typing import Any, Dict, List, Optional

from pydantic import BaseModel


class MetaActionData(BaseModel):
    """
    action status in grutopia
    """
    controller: str
    data: Any


class ActionData(BaseModel):
    """
    action status in grutopia
    """
    robot: str
    controllers: List[MetaActionData]


class _IsaacData(BaseModel):
    """
    isaac status in grutopia
    """
    actions: Optional[List[Dict[str, Any]]]
    obs: Optional[List[Dict[str, Any]]]


class IsaacData:
    """
    isaac status in grutopia

    There are two types of isaac status:

    * Action
    * Observation

    structure of isaac status like this::

            {
                actions: {
                    [
                        {
                            robot_1: {
                                cap: param,
                            }
                        }
                    ]
                },
                observations: {
                    [
                        {
                            robot_1: {
                                obs_1: data,
                                obs_2: data
                            }
                        }
                    ]
                }
            }

    """
    data = _IsaacData(actions=[], obs=[])

    def __init__(self) -> None:
        pass

    @classmethod
    def get_all(cls) -> _IsaacData:
        return cls.data

    # Observation
    @classmethod
    def set_obs_data(cls, obs: List[Dict[str, Any]]) -> None:
        cls.data.obs = obs

    @classmethod
    def get_obs(cls) -> List[Dict[str, Any]]:
        """
        Get isaac observation data

        Returns:
            isaac observation data list
        """
        return cls.data.obs

    @classmethod
    def get_obs_by_id(cls, task_id: int) -> Dict[str, Any]:
        """
        Get isaac observation by id

        Args:
            task_id: isaac task id

        Returns:
            isaac observation data

        """
        return cls.data.obs[task_id]

    # Action
    @classmethod
    def add_actions(cls, actions: List[ActionData]):
        """
        Add actions

        Args:
            actions: action list

        Returns:

        """
        # when add action, return action's index.
        cls.data.actions = []
        for action in actions:
            cls.data.actions.append({action.robot: {x.controller: x.data for x in action.controllers}})
        return

    @classmethod
    def get_actions(cls) -> None | List[Dict[Any, Any]]:
        """
        Get actions

        Returns:
            action(dict like {robot_name: {controller_name: param}}) list
        """
        return cls.data.actions

    @classmethod
    def get_action_by_id(cls, task_id: int) -> None | Dict[Any, Any]:
        """
        Get action by id

        Returns:
            action(dict like {robot_name: {controller_name: param}})
        """
        return cls.data.actions[task_id]
