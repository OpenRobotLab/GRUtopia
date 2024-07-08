"""
Includes web ui interactive
"""
import datetime
import os
from typing import Any, Dict, Union

from grutopia.core.datahub.web_api import WebBEUrl
from grutopia.core.util import AsyncRequest

# constants
SendChatControlUrl = WebBEUrl + '/api/grutopia/append_chat_control_data'
SendCOTUrl = WebBEUrl + '/api/grutopia/append_chain_of_thought_data'
SendLogDataUrl = WebBEUrl + '/api/grutopia/append_log_data'
GetChatControlUrl = WebBEUrl + '/api/grutopia/getChatList'
GetLogDataUrl = WebBEUrl + '/api/grutopia/getloglist'
ClearUrl = WebBEUrl + '/api/grutopia/clear'

WEBUI_HOST = os.getenv('WEBUI_HOST', '127.0.0.1')

DefaultAvatarUrl = f'http://{WEBUI_HOST}:8080/static/avatar_default.jpg'

AvatarUrls = {
    'user': f'http://{WEBUI_HOST}:8080/static/avatar_00.jpg',
    'agent': f'http://{WEBUI_HOST}:8080/static/avatar_01.jpg',
}


def send_chain_of_thought(cot: str, uuid: str = 'none') -> None:
    """
    chain of thought data

    Args:
        uuid (str): uuid of chain of thought data, defaults to "none".
        cot (str): chain of thought data.
    """

    def cot_format(x):
        return {'type': 'text', 'value': x}

    res_data = [{'type': 'time', 'value': datetime.datetime.now().strftime('%H:%M')}]
    for i in cot:
        res_data.append(cot_format(i))
    AsyncRequest.post(uuid, SendCOTUrl, res_data)


def send_chat_control(nickname: str, text: str, img: str = None, role: str = 'user', uuid: str = 'none') -> None:
    """Send a new message to the chatbox.

    Args:
        nickname (str): nickname displayed in the chatbox.
        text (str): text to send to the chatbox.
        img (str, optional): image to send to the chatbox. Defaults to None.
        role (str, optional): role name, user or agent. Defaults to "user".
        uuid (str, optional): uuid of the message. Defaults to 'none'.
    """
    avatar_url = AvatarUrls.get(role, DefaultAvatarUrl)
    res_data = {
        'type': role,
        'name': nickname,
        'time': datetime.datetime.now().strftime('%H:%M'),
        'message': text,
        'photo': avatar_url,
        'img': img,
    }
    AsyncRequest.post(uuid, SendChatControlUrl, res_data)


def send_log_data(log_data: str,
                  log_type: str = 'message',
                  user: str = 'Bob',
                  photo_url: str = DefaultAvatarUrl,
                  uuid: str = 'none') -> None:
    """Send log data.

    Args:
        uuid (str): uuid of log, default is none.
        log_data (str): log data.
        log_type (str): type of log. 'message' or 'user'.
        user (str): logger name. default: Bob.
        photo_url (str): log photo url path.

    """
    if log_type == 'message':
        res_data = {'type': 'message', 'message': log_data}
    else:  # user
        if log_type != 'user':
            return
        res_data = {
            'type': 'user',
            'name': user,
            'time': datetime.datetime.now().strftime('%H:%M'),
            'message': log_data,
            'photo': photo_url
        }
    AsyncRequest.post(uuid, SendLogDataUrl, res_data)


def get_log_data(uuid: str = 'none') -> Union[Dict[str, Any], None]:
    """
    Get log data.

    Args:
        uuid (str): log data uuid. default: none.

    Returns:
        log_data (list[dict]): log data.
    """
    ok, json_data = AsyncRequest.get(uuid, GetLogDataUrl)
    if ok and json_data is not None:
        return json_data
    return None


def get_chat_control(uuid: str = 'none') -> Union[Dict[str, Any], None]:
    """
    Get chat control data.

    Args:
        uuid (str): chat control uuid. default: none.

    Returns:
        chat_control (List[Dict[str, Any]]): chat control data.
    """
    ok, json_data = AsyncRequest.get(uuid, GetChatControlUrl)
    if ok and json_data is not None:
        return json_data
    return None


def clear(uuid: str = 'none'):
    """
    Clear all data in webui.
    """
    AsyncRequest.post(uuid, ClearUrl, None)
