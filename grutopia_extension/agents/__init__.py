import importlib

from grutopia.core.util import log

for module in [
    'grutopia_extension.agents.dummy_agent',
    'grutopia_extension.agents.npc_agent_client',
    'grutopia_extension.agents.social_navigation_agent_client',
    'grutopia_extension.agents.mobile_manipulation_agent_client',
]:
    try:
        importlib.import_module(module)
    except ImportError as e:
        log.error(e)
        continue
