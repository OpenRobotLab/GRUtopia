from typing import Any, Dict

from fastapi import APIRouter

from grutopia.core.datahub import ActionData, IsaacData

router = APIRouter(prefix='/api/isaac')


@router.get('/get_obs_by_task_name/{task_name}', tags=['Isaac Interactive'])
def get_obs_by_task_name(task_name: str) -> Dict[str, Any]:
    """
    Get Isaac Observation by task id web API.

    Args:
        task_name(str): task name

    Returns:
        Observation data.
    """
    return {'msg': 'OK', 'data': IsaacData.get_obs_by_task_name(task_name)}


@router.get('/get_obs_by_task_name_and_robot_name/{task_name}/{robot_name}', tags=['Isaac Interactive'])
def get_obs_by_task_name_and_robot_name(task_name: str, robot_name: str) -> Dict[str, Any]:
    """
    Get Isaac Observation by task id web API.

    Args:
        robot_name (str): robot name
        task_name(str): task name

    Returns:
        Observation data.
    """
    return {'msg': 'OK', 'data': IsaacData.get_obs_by_task_name_and_robot_name(task_name, robot_name)}


# TODO Refactor this
@router.post('/set_obs', tags=['Isaac Interactive'])
def set_obs(obs: Dict[str, Any]) -> dict[str, str]:
    """
    Flush observation data web API.

    Args:
        obs(Dict[str, Any]): data to be flushed.

    Returns:
        OK if flush successfully.
    """
    IsaacData.set_obs_data(obs)
    return {'msg': 'OK'}


@router.get('/get_action_by_id/{task_name}', tags=['Isaac Interactive'])
def get_actions_by_task_name(task_name: str) -> dict:
    """
    get action by task name

    Args:
        task_name(str): action of task{task_name}

    Returns:
        action's info dict
    """
    return {'msg': 'OK', 'data': IsaacData.get_action_by_task_name(task_name)}


@router.post('/set_actions/', tags=['Isaac Interactive'])
def set_actions(actions: Dict[str, ActionData]) -> dict:
    """
    get action by id

    Args:
        actions(str): action of task{task_name}

    Returns:
        action's info dict
    """

    IsaacData.set_actions(actions)
    return {'msg': 'OK'}
