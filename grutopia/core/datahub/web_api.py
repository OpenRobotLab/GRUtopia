"""
Includes web api endpoints
"""
from typing import Any, Dict, List

import httpx

from grutopia.core.datahub.isaac_data import ActionData

# constants
WebBEUrl = 'http://127.0.0.1:9000'  # TODO config this
GetAllObsPath = WebBEUrl + '/api/stream/get_all_obs'
GetObsByIdPath = WebBEUrl + '/api/stream/get_obs_by_id/'
FlushObsUrl = WebBEUrl + '/api/isaac/flush_obs_data'
SetActionsUrl = WebBEUrl + '/api/isaac/set_action'
GetAllActionUrl = WebBEUrl + '/api/isaac/get_actions'
GetActionByIdUrl = WebBEUrl + '/api/isaac/get_action_by_id/'


def get_all_obs() -> List[Dict[str, Any]] | None:
    """
    Get all observation data
    Returns:
        obs (List[Dict[str, Any]]): List of all observation data
    """
    r = httpx.get(GetAllObsPath)
    if r.status_code == 200:
        return r.json()
    return None


def get_obs_by_id(task_id: int) -> Any | None:
    """
    Get observation by id
    Args:
        task_id (int): id of observation data

    Returns:
        obs (Any): Observation data
    """
    r = httpx.get(GetObsByIdPath + str(task_id))
    if r.status_code == 200:
        return r.json()


def set_obs_data(obs: List[Dict[str, Any]]) -> bool:
    """
    Set observation data web API
    Args:
        obs (List[Dict[str, Any]]): isaac observation data

    Returns:
        OK if set successfully
    """
    r = httpx.post(FlushObsUrl, json=obs, timeout=1)
    if r.status_code == 200 and r.json()['msg'] == 'OK':
        return True
    return False


# Action
# send get, no poll&callback(all depends on ).
def get_actions():
    r = httpx.get(GetAllActionUrl)
    if r.status_code == 200 and r.json()['data'] is not None:
        return r.json()['msg'], r.json()['data']
    return None, {}


def get_actions_by_id(task_id: int):
    """
    Get actions by task id(int)

    Args:
        task_id(int): id of task

    Returns:
        msg: msg str(or None)
        data: data
    """
    r = httpx.get(GetActionByIdUrl + str(task_id))
    if r.status_code == 200 and r.json()['data'] is not None:
        return r.json()['msg'], r.json()['data']
    return None, {}


def send_actions(actions: List[ActionData]) -> bool:
    """
    send actions
    Args:
        actions(List[ActionData]): action data list

    Returns:
        Send message successfully or not
    """
    r = httpx.post(SetActionsUrl, json=actions)
    if r.status_code == 200:
        return True
    return False
