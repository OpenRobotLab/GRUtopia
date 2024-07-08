import os

from .log import log


def is_in_container() -> bool:
    try:
        os.stat('/.dockerenv')
        return True
    except FileNotFoundError:
        return False
    except Exception as e:
        log.error(f'Error while checking if in container, assume False: {e}')
        return False
