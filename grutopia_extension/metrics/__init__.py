import importlib

from grutopia.core.util import log

for module in [
    'grutopia_extension.metrics.simple_metric',
    'grutopia_extension.metrics.recording_metric',
]:

    try:
        importlib.import_module(module)
    except ImportError as e:
        log.error(e)
