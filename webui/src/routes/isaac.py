from typing import Any, Dict, List, Union

from fastapi import APIRouter

from grutopia.core.datahub import ActionData, IsaacData

router = APIRouter(prefix='/api/isaac')


@router.get('/get_all_obs', tags=['Isaac Interactive'])
def get_all_obs():
    """
    Get all Isaac Observation data web API.

    Returns:
        Observation data list.
    """
    return IsaacData.get_obs()


@router.get('/get_obs_by_id/{task_id}', tags=['Isaac Interactive'])
def get_obs_by_id(task_id: int) -> Dict[str, Any]:
    """
    Get Isaac Observation by task id web API.

    Args:
        task_id(int): task id in env

    Returns:
        Observation data.
    """
    return IsaacData.get_obs_by_id(task_id)


@router.post('/flush_obs_data', tags=['Isaac Interactive'])
def flush_obs_data(obs: List[Dict[str, Any]]) -> dict[str, str]:
    """
    Flush observation data web API.

    Args:
        obs(List[Dict[str, Any]]): data to be flushed.

    Returns:
        OK if flush successfully.
    """
    IsaacData.set_obs_data(obs)
    return {'msg': 'OK'}


@router.post('/set_action', tags=['Isaac Interactive'])
def add_action(action: List[ActionData]) -> dict[str, Union[str, dict[str, Any]]]:
    """
    Set action

    Args:
        action(List[ActionData]): actions data

    Returns:
        OK if action successfully or not.
    """
    IsaacData.add_actions(action)
    return {'msg': 'OK'}


@router.get('/get_actions', tags=['Isaac Interactive'])
def get_actions() -> dict:
    """
    get all actions

    Returns:
        actions' info dict
    """
    return {'msg': 'OK', 'data': IsaacData.get_actions()}


@router.get('/get_action_by_id/{task_id}', tags=['Isaac Interactive'])
def get_actions_by_id(task_id: int) -> dict:
    """
    get action by id

    Args:
        task_id: action of task{task_id}

    Returns:
        action's info dict
    """
    return {'msg': 'OK', 'data': IsaacData.get_action_by_id(task_id)}
