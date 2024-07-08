from grutopia.core.util.async_req import AsyncRequest
from grutopia.core.util.log import log

if not AsyncRequest.loop:
    AsyncRequest.start_loop()
