import os

from grutopia.core.util.async_req import AsyncRequest
from grutopia.core.util.log import log

if not AsyncRequest.loop:
    AsyncRequest.start_loop()


def is_in_container() -> bool:
    try:
        os.stat('/.dockerenv')
        return True
    except FileNotFoundError:
        return False
    except Exception as e:
        log.error(f'Error while checking if in container, assume False: {e}')
        return False


def has_display() -> bool:
    try:
        os.environ['DISPLAY']
        return True
    except KeyError:
        return False
    except Exception as e:
        log.error(f'Error while checking if has display, assume False: {e}')
        return False
