import importlib

from internutopia.core.util import log

for module in [
    'internutopia_extension.metrics.simple_metric',
    'internutopia_extension.metrics.recording_metric',
]:

    try:
        importlib.import_module(module)
    except ImportError as e:
        log.error(e)
