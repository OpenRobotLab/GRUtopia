import os

from addict import Dict


class MacroDict(Dict):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self['_read'] = set()

    def __setattr__(self, name, value):
        if name in self.get('_read', set()):
            raise AttributeError(f'Cannot set attribute {name} in MacroDict, it has already been used.')
        # Use the super's setattr for setting attributes, but handle _read directly to avoid recursion.
        if name == '_read':
            self[name] = value
        else:
            super().__setattr__(name, value)

    def __getattr__(self, item):
        # Directly check and modify '_read' to avoid going through __getattr__ or __setattr__.
        if item != '_read':
            self['_read'].add(item)
        # Use direct dictionary access to avoid infinite recursion.
        try:
            return self[item]
        except KeyError:
            raise AttributeError(f"'MacroDict' object has no attribute '{item}'")


# Initialize settings
macros = MacroDict()
gm = macros.globals


def determine_gm_path(default_path, env_var_name):
    # Start with the default path
    path = default_path
    # Override with the environment variable, if set
    if env_var_name in os.environ:
        path = os.environ[env_var_name]
    # Expand the user directory (~)
    path = os.path.expanduser(path)
    # Make the path absolute if it's not already
    if not os.path.isabs(path):
        path = os.path.join(os.path.dirname(os.path.realpath(__file__)), path)
    return path


# Users can override the path if needed
gm.ASSET_PATH = determine_gm_path('../assets', 'GRUTOPIA_ASSET_PATH')
