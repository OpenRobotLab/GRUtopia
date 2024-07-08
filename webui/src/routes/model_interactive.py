import asyncio
import datetime
from threading import Thread
from typing import Annotated, Any, Dict, List

import httpx
from fastapi import APIRouter, Body
from pydantic import BaseModel

from grutopia.core.datahub.model_data import ChainOfThoughtDataItem, ChatControlData, LogData, ModelData
from grutopia.core.datahub.web_ui_api import WEBUI_HOST
from grutopia.core.util import log
from webui.src.routes.response_model import SuccessModel

router = APIRouter(prefix='/api/grutopia')

message_send_loop = asyncio.new_event_loop()


def start_loop(loop):
    asyncio.set_event_loop(loop)
    loop.run_forever()


t = Thread(target=start_loop, args=(message_send_loop, ))
t.daemon = True
t.start()


@router.get('/getloglist', tags=['Model Interactive'])
async def get_log_data() -> Dict[str, Any]:
    return SuccessModel(ModelData.get_log_data()).get_res_body()


@router.get('/getThoughtChain', tags=['Model Interactive'])
async def get_chain_of_thought_data() -> Dict[str, Any]:
    return SuccessModel(ModelData.get_chan_of_thought()).get_res_body()


@router.get('/getChatList', tags=['Model Interactive'])
async def get_chat_control_data() -> Dict[str, Any]:
    return SuccessModel(ModelData.get_chat_control()).get_res_body()


@router.post('/append_log_data', tags=['Model Interactive'])
async def append_log_data(
    data: Annotated[LogData,
                    Body(examples=[{
                        'type': 'user',
                        'name': 'Bob',
                        'time': '17:20',
                        'message': 'This image shows a blue computer screen with a white background.',
                        'photo': 'https://static.openxlab.org.cn/puyu/demo/000-2x.jpg'
                    }, {
                        'type': 'message',
                        'message': '- 15:16:30 Agent moves to the counter -'
                    }, {
                        'type': 'agent',
                        'name': 'Agent',
                        'time': '18:20',
                        'message': 'Hi, Understand this picture',
                        'photo': 'https://static.openxlab.org.cn/puyu/demo/000-2x.jpg'
                    }])]
) -> Dict[str, Any]:  # noqa E125
    ModelData.append_log_data(data)
    return SuccessModel(None, msg='log_data appended').get_res_body()


@router.post('/append_chain_of_thought_data', tags=['Model Interactive'])
async def append_chain_of_thought_data(
    data: Annotated[List[ChainOfThoughtDataItem],
                    Body(examples=[[{
                        'type': 'text',
                        'value': '1.ask the staff where I can find the shoes'
                    }, {
                        'type': 'text',
                        'value': '2.find a pair of white shoes'
                    }, {
                        'type': 'text',
                        'value': '3.pick up the shoes'
                    }, {
                        'type': 'text',
                        'value': '4.take it to the counter'
                    }, {
                        'type': 'text',
                        'value': '5.payment'
                    }, {
                        'type': 'time',
                        'value': '15:16'
                    }]])]
) -> Dict[str, Any]:
    ModelData.append_chan_of_thought(data)
    return SuccessModel(None, msg='chain_of_thought_data appended').get_res_body()


@router.post('/append_chat_control_data', tags=['Model Interactive'])
async def append_chat_control_data(
    data: Annotated[ChatControlData,
                    Body(examples=[{
                        'type': 'user',
                        'name': 'Bob',
                        'time': '17:20',
                        'message': 'This image shows a blue computer screen with a white background.',
                        'photo': 'https://static.openxlab.org.cn/puyu/demo/000-2x.jpg',
                        'img': 'https://static.openxlab.org.cn/puyu/demo/000-2x.jpg'
                    }, {
                        'type': 'message',
                        'message': '- 15:16:30 Agent moves to the counter -'
                    }, {
                        'type': 'message',
                        'message': '- 15:16:50 Agent picks up the white shoes -'
                    }, {
                        'type': 'agent',
                        'name': 'Agent',
                        'time': '18:20',
                        'message': 'Hi, Understand this picture',
                        'photo': 'https://static.openxlab.org.cn/puyu/demo/000-2x.jpg',
                        'img': 'https://static.openxlab.org.cn/puyu/demo/000-2x.jpg'
                    }, {
                        'type': 'agent',
                        'name': 'Agent',
                        'time': '19:20',
                        'photo': 'https://static.openxlab.org.cn/puyu/demo/000-2x.jpg',
                        'status': 'pending'
                    }, {
                        'type': 'user',
                        'name': 'Bob',
                        'time': '20:20',
                        'message': 'This image shows a blue computer screen with a white background.',
                        'photo': 'https://static.openxlab.org.cn/puyu/demo/000-2x.jpg',
                        'img': 'https://static.openxlab.org.cn/puyu/demo/000-2x.jpg'
                    }])]
) -> Dict[str, Any]:  # noqa E125
    ModelData.append_chat_control(data)
    return SuccessModel(None, msg='chat_control_data appended').get_res_body()


@router.post('/clear', tags=['Model Interactive'])
async def clear():
    ModelData.clear()
    return SuccessModel(None, msg='all cleared').get_res_body()


class MsgModel(BaseModel):
    prompt: str
    imgUrl: str


async def send_chat_control(text: str, img: str):
    r = httpx.post('http://127.0.0.1:9999/model/chat', json={'text': text, 'img': img})
    if r.status_code != 200:
        log.error('Send_chat_control Fail')


@router.post('/sendMessage', tags=['Model Interactive'])
async def send_message_interactive(
    data: Annotated[MsgModel,
                    Body(examples=[{
                        'message': 'This image shows a blue computer screen with a white background.',
                        'img': 'https://static.openxlab.org.cn/puyu/demo/000-2x.jpg'
                    }, {
                        'message': 'This image shows a blue computer screen with a white background.',
                        'img': 'https://static.openxlab.org.cn/puyu/demo/000-2x.jpg'
                    }])]
) -> Dict[str, Any]:  # noqa E125
    log.debug(data.prompt)
    ModelData.append_chat_control(
        ChatControlData(type='user',
                        name='Agent',
                        time=datetime.datetime.now().strftime('%H:%M'),
                        message=data.prompt,
                        photo=f'http://{WEBUI_HOST}:8080/static/avatar_00.jpg',
                        img=data.imgUrl))
    asyncio.run_coroutine_threadsafe(send_chat_control(data.prompt, data.imgUrl), message_send_loop)
    return SuccessModel({}).get_res_body()
