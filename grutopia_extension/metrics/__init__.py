import importlib

from grutopia.core.util import log

for module in [
        'grutopia_extension.metrics.candidates_reduce_metric', 'grutopia_extension.metrics.debug_metric',
        'grutopia_extension.metrics.mobile_manipulation_success_metric', 'grutopia_extension.metrics.reset_time_metric',
        'grutopia_extension.metrics.simple_metric', 'grutopia_extension.metrics.social_navigation_success_metric'
]:

    try:
        importlib.import_module(module)
    except ImportError as e:
        log.error(e)
