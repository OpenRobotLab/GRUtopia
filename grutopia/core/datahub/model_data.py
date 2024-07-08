from typing import List, Optional

from pydantic import BaseModel, Field


class LogData(BaseModel):
    type: str = Field(..., example='agent', description='message(notification)|agent(response)|user(question)')
    name: Optional[str] = Field(None, example='Bob', description='nickname')
    time: Optional[str] = Field(None, example='17:20', description='time of the message')
    message: str = Field(...,
                         example='his image shows a blue computer screen with a white background.',
                         description='text content of the message')
    photo: Optional[str] = Field(None,
                                 example='https://static.openxlab.org.cn/puyu/demo/000-2x.jpg',
                                 description='avatar url')


class ChainOfThoughtDataItem(BaseModel):
    type: str = Field(..., example='text', description='time(notification)|message(normal message)')
    value: str = Field(..., example='I need to do the following things', description='text content of the message')


class ChatControlData(BaseModel):
    type: str = Field(..., example='agent', description='message(notification)|agent(response)|user(question)')
    name: Optional[str] = Field(None, example='Agent', description='nickname')
    time: Optional[str] = Field(None, example='18:20', description='time of the message')
    message: Optional[str] = Field(None,
                                   example='Hi, Understand this picture',
                                   description='text content of the message')
    photo: Optional[str] = Field(None,
                                 example='https://static.openxlab.org.cn/puyu/demo/000-2x.jpg',
                                 description='avatar url')
    img: Optional[str] = Field(None,
                               example='https://static.openxlab.org.cn/puyu/demo/000-2x.jpg',
                               description='image content of the message')
    status: Optional[str] = Field(None, example='pending', description='status of the message, legal values: pending')


class ModelData:
    data = {
        'log_data': [],
        'chain_of_thought_data': [],
        'chat_control_data': [],
    }

    def __init__(self) -> None:
        pass

    @classmethod
    def get_log_data(cls):
        return cls.data['log_data']

    @classmethod
    def get_chan_of_thought(cls):
        return cls.data['chain_of_thought_data']

    @classmethod
    def get_chat_control(cls):
        return cls.data['chat_control_data']

    @classmethod
    def append_log_data(cls, data: LogData):
        return cls.data['log_data'].append(data.dict())

    @classmethod
    def append_chan_of_thought(cls, data: List[ChainOfThoughtDataItem]):
        return cls.data['chain_of_thought_data'].append([d.dict() for d in data])

    @classmethod
    def append_chat_control(cls, data: ChatControlData):
        return cls.data['chat_control_data'].append(data.dict())

    @classmethod
    def clear(cls):
        cls.data['chat_control_data'].clear()
        cls.data['chain_of_thought_data'].clear()
        cls.data['log_data'].clear()
