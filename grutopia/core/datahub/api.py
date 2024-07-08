from typing import Any, Dict, List

from grutopia.core.datahub.isaac_data import ActionData, IsaacData


def get_all_obs() -> List[Dict[str, Any]]:
    """
    Get all observation data.

    Returns:
        List[Dict[str, Any]]: sensor data dict
    ```
    """
    return IsaacData.get_obs()


def get_obs_by_id(task_id: int) -> Dict[str, Any]:
    """
    Get observation by task_id

    Returns:
        Dict[str, Any]: obs data dict
    """
    return IsaacData.get_obs_by_id(task_id)


def set_obs_data(obs: List[Dict[str, Any]]) -> None:
    """
    Flush observation data.

    Args:
        obs (List[Dict[str, Any]]): observation data

    """
    IsaacData.set_obs_data(obs)


def get_actions() -> None | Dict[Any, Any]:
    """
    Get all actions

    Returns:
        Dict[str, Any]: action data dict
    """
    return IsaacData.get_actions()


def send_actions(actions: List[ActionData]):
    """
    send actions to datahub
    Args:
        actions (List[ActionData]): list of [dict of {robot_id: ActionData}]
    """
    IsaacData.add_actions(actions)
