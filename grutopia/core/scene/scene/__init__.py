from grutopia.core.scene.scene.util import usd_op


def create_scene(config_json_path: str, prim_path_root: str = 'background'):
    """
    TODO: rename.
    Create a scene from config.(But just input usd file yet.)
    Args:
        config_json_path (str): path to scene config file(use to be a .usd file)
        prim_path_root (str): path to root prim

    Returns:
        config_json_path (str): path to config file
        world_prim_path (str): path to world prim
    """
    world_prim_path = '/' + prim_path_root
    if config_json_path.endswith('usd') or config_json_path.endswith('usda') or config_json_path.endswith('usdc'):
        # Add usd directly
        return config_json_path, world_prim_path
    raise RuntimeError('Env file path needs to end with .usd, .usda or .usdc .')
