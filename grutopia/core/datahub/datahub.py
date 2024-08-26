import datetime
from typing import Any, Dict, List

import httpx

from grutopia.core.datahub.isaac_data import ActionData, IsaacData
from grutopia.core.datahub.model_data import ChainOfThoughtDataItem, ChatControlData, LogData, ModelData
from grutopia.core.util import AsyncRequest


class DataHub:
    """
    DataHub provides methods for interacting with Isaac Data.

    It contains:

    **Task**
       - Get task_name(): str
    **Robot**
       - Get action by task name
       - Set action by task name
       - Get obs by task name
       - Get obs by task name and robot name
       - Set obs by task name
    **Episode**
       - Get if episode finished
       - Set if episode finished
    **Webui chat**
       - Get chat log
       - Get chat control
       - Append chat log
       - Append chat control
       - Append cot

    """

    sim_remote = False
    chat_remote = True
    remote_address = ''
    WebBEUrl = ''
    GetAllObsUrl = ''
    SetObsUrl = ''
    GetObsByTaskNameUrl = ''
    SetActionsUrl = ''
    GetActionByTaskNameUrl = ''
    GenTaskIdUrl = ''
    SendChatControlUrl = ''
    SendCoTUrl = ''
    SendLogDataUrl = ''
    GetChatControlUrl = ''
    GetLogDataUrl = ''
    ClearUrl = ''
    AvatarUrls = {}
    DefaultAvatarUrl = ''

    @classmethod
    def datahub_init(
        cls,
        sim: str = 'local',
        chat: str = 'remote',
        remote: str = '127.0.0.1:9000',
    ):
        cls.sim_remote = sim == 'remote'
        cls.chat_remote = chat == 'remote'
        cls.remote_address = remote
        cls.set_attr()

    @classmethod
    def set_attr(cls) -> None:
        # BE url path init
        cls.WebBEUrl = f'http://{cls.remote_address}'  # noqa
        cls.GetAllObsUrl = cls.WebBEUrl + '/api/stream/get_all_obs'

        cls.SetObsUrl = cls.WebBEUrl + '/api/isaac/set_obs/'
        cls.GetObsByTaskNameUrl = cls.WebBEUrl + '/api/isaac/get_obs_by_id/'
        cls.GetObsByTaskNameAndRobotNameUrl = cls.WebBEUrl + '/api/isaac/get_obs_by_task_name_and_robot_name/'
        cls.SetActionsUrl = cls.WebBEUrl + '/api/isaac/set_actions/'
        cls.GetActionByTaskNameUrl = cls.WebBEUrl + '/api/isaac/get_action_by_id/'

        cls.GenTaskIdxUrl = cls.WebBEUrl + '/api/isaac/gen_task_idx'

        cls.SendChatControlUrl = cls.WebBEUrl + '/api/grutopia/append_chat_control_data'
        cls.SendCoTUrl = cls.WebBEUrl + '/api/grutopia/append_chain_of_thought_data'
        cls.SendLogDataUrl = cls.WebBEUrl + '/api/grutopia/append_log_data'
        cls.GetChatControlUrl = cls.WebBEUrl + '/api/grutopia/getChatList'
        cls.GetLogDataUrl = cls.WebBEUrl + '/api/grutopia/getloglist'

        cls.ClearUrl = cls.WebBEUrl + '/api/grutopia/clear'

        cls.AvatarUrls = {
            'user': f'http://{cls.remote_address.split(":")[0]}:8080/static/avatar_00.jpg',  # noqa
            'agent': f'http://{cls.remote_address.split(":")[0]}:8080/static/avatar_01.jpg',  # noqa
        }
        cls.DefaultAvatarUrl = f'http://{cls.remote_address.split(":")[0]}:8080/static/avatar_default.jpg'  # noqa

    def __init__(self):
        pass

    @classmethod
    def get_actions_by_task_name(cls, task_name: str) -> Dict[str, Dict]:
        """
        Get action by task name.
        TODO async
        Args:
            task_name (str): task name

        Returns:
            Action of this
        """
        if cls.sim_remote:
            r = httpx.get(cls.GetActionByTaskNameUrl + task_name)
            if r.status_code == 200:
                return r.json()
        else:
            return IsaacData.get_action_by_task_name(task_name)

    @classmethod
    def get_obs_by_task_name(cls, task_name: str) -> Dict:
        """
        Get observation by task name
        TODO Async

        Args:
            task_name (str): Task name

        Returns:
            obs (Any): Observation data
        """
        if cls.sim_remote:
            r = httpx.get(cls.GetObsByTaskNameUrl + task_name)
            if r.status_code == 200:
                return r.json()
        else:
            return IsaacData.get_obs_by_task_name(task_name)

    @classmethod
    def get_obs_by_task_name_and_robot_name(cls, task_name: str, robot_name: str) -> Dict:
        """
        Get observation by task name
        TODO Async

        Args:
            robot_name (str): robot name
            task_name (str): Task name

        Returns:
            obs (Any): Observation data
        """
        if cls.sim_remote:
            r = httpx.get(cls.GetObsByTaskNameAndRobotNameUrl + task_name + '/' + robot_name)
            if r.status_code == 200:
                return r.json()
        else:
            return IsaacData.get_obs_by_task_name_and_robot_name(task_name, robot_name)

    @classmethod
    def set_obs_data(cls, obs: Dict[str, Dict[str, Any]]):
        """
        Set observation
        TODO Async

        Args:
            obs (Dict[str, Dict[str, Any]]): Observation data with task_name as key.
        """
        if cls.sim_remote:
            r = httpx.post(cls.SetObsUrl, )
            if r.status_code == 200:
                return r.json()
        else:
            IsaacData.set_obs_data(obs)

    @classmethod
    def set_actions(cls, actions: Dict[str, ActionData]):
        """
        Set actions (dict, task_name(str) as key)
        TODO Async, Robot-wise

        Args:
            actions (Dict[str, ActionData]): Action data (dict) with task_name as key
        """
        if cls.sim_remote:
            httpx.post(cls.SetActionsUrl, json={'actions': actions})
        else:
            IsaacData.set_actions(actions)

    @classmethod
    def get_episode_finished(cls, task_name: str) -> bool:
        """
        Get if episode finish by task name.

        Args:
            task_name (str): task name.

        Returns:
            bool: status of the task (finished or not)
        """
        if cls.sim_remote:
            # TODO Implement this
            raise NotImplementedError('get_episode_finish NotImplementedError')
        else:
            return IsaacData.get_episode_finished(task_name)

    @classmethod
    def set_episode_finished(cls, task_name: str):
        """
        Set episode finished by task_name.

        Args:
            task_name (str): task_name.
        """
        if cls.sim_remote:
            # TODO Implement this
            pass
        else:
            IsaacData.set_episode_finished(task_name)

    @classmethod
    def gen_task_idx(cls) -> str:
        """
        Generate task index for new task.

        Returns:
            str: task index for the new task.
        """
        if cls.sim_remote:
            r = httpx.get(cls.GenTaskIdxUrl)
            if r.status_code == 200 and r.json()['data'] is not None:
                return r.json()['data']
            raise RuntimeError(f'Failed to get task_name, r.json: {r.json()}')
        else:
            return IsaacData.gen_task_idx()

    @classmethod
    def send_chain_of_thought(cls, cot: str, uuid: str = 'none') -> None:
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
        if cls.chat_remote:
            AsyncRequest.post(uuid, cls.SendCoTUrl, res_data)
        else:
            ModelData.append_chan_of_thought([ChainOfThoughtDataItem(**i) for i in res_data])

    @classmethod
    def send_chat_control(cls,
                          nickname: str,
                          text: str,
                          img: str = None,
                          role: str = 'user',
                          uuid: str = 'none') -> None:
        """Send a new message to the chatbox.

        Args:
            nickname (str): nickname displayed in the chatbox.
            text (str): text to send to the chatbox.
            img (str, optional): image to send to the chatbox. Defaults to None.
            role (str, optional): role name, user or agent. Defaults to "user".
            uuid (str, optional): uuid of the message. Defaults to 'none'.
        """
        avatar_url = cls.AvatarUrls.get(role, cls.DefaultAvatarUrl)
        res_data = {
            'type': role,
            'name': nickname,
            'time': datetime.datetime.now().strftime('%H:%M'),
            'message': text,
            'photo': avatar_url,
            'img': img,
        }
        if cls.chat_remote:
            AsyncRequest.post(uuid, cls.SendChatControlUrl, res_data)
        else:
            ModelData.append_chat_control(ChatControlData(**res_data))

    @classmethod
    def send_log_data(cls,
                      log_data: str,
                      log_type: str = 'message',
                      user: str = 'Bob',
                      photo_url: str = None,
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
            if photo_url is None:
                photo_url = cls.DefaultAvatarUrl
            if log_type != 'user':
                return
            res_data = {
                'type': 'user',
                'name': user,
                'time': datetime.datetime.now().strftime('%H:%M'),
                'message': log_data,
                'photo': photo_url
            }
        if cls.chat_remote:
            AsyncRequest.post(uuid, cls.SendLogDataUrl, res_data)
        else:
            ModelData.append_log_data(LogData(**res_data))

    @classmethod
    def get_log_data(cls, uuid: str = 'none') -> List[Dict]:
        """
        Get log data.

        Args:
            uuid (str): log data uuid. default: none.

        Returns:
            log_data (List[Dict]): log data.
        """
        if cls.chat_remote:
            ok, json_data = AsyncRequest.get(uuid, cls.GetLogDataUrl)
            if ok and json_data is not None and 'data' in json_data:
                return json_data['data']
            return []
        return ModelData.get_log_data()

    @classmethod
    def get_chat_control(cls, uuid: str = 'none') -> List[Dict[str, Any]]:
        """
        Get chat control data.

        Args:
            uuid (str): chat control uuid. default: none.

        Returns:
            chat_control (List[Dict[str, Any]]): chat control data.
        """
        if cls.chat_remote:
            ok, json_data = AsyncRequest.get(uuid, cls.GetChatControlUrl)
            if ok and json_data is not None and 'data' in json_data:
                return json_data['data']
            return []
        else:
            return ModelData.get_chat_control()

    @classmethod
    def clear(cls, uuid: str = 'none'):
        """
        Clear all data in webui.
        """
        if cls.chat_remote:
            AsyncRequest.post(uuid, cls.ClearUrl, None)
        else:
            ModelData.clear()
