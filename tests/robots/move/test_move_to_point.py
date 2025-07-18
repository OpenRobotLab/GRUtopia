from copy import deepcopy
from typing import Any, Dict, Tuple

import numpy as np

from grutopia.core.config import Config, RobotCfg, SimConfig
from grutopia.core.gym_env import Env
from grutopia.core.util import has_display
from grutopia.macros import gm
from grutopia_extension import import_extensions
from grutopia_extension.configs.tasks import SingleInferenceTaskCfg

headless = not has_display()


def run(robot: RobotCfg, action: Dict[str, Any], target: Tuple[float, float, float], max_steps=5000):
    config = Config(
        simulator=SimConfig(
            physics_dt=1 / 240, rendering_dt=1 / 240, use_fabric=True, headless=headless, native=headless
        ),
        task_configs=[
            SingleInferenceTaskCfg(
                scene_asset_path=gm.ASSET_PATH + '/scenes/empty.usd',
                robots=[robot.update()],
            ),
            SingleInferenceTaskCfg(
                scene_asset_path=gm.ASSET_PATH + '/scenes/empty.usd',
                robots=[robot.update()],
            ),
            SingleInferenceTaskCfg(
                scene_asset_path=gm.ASSET_PATH + '/scenes/empty.usd',
                robots=[robot.update()],
            ),
        ],
    )

    import_extensions()

    env = Env(config)
    obs, _ = env.reset()

    i = 0
    episode_idx = 0

    env_action = action

    print(f'actions: {env_action}')
    solid = 0

    while env.simulation_app.is_running():
        i += 1
        obs, _, _, _, _ = env.step(action=env_action)

        position = deepcopy(obs['position'])
        position[2] = target[2]  # Make sure we ignore z axis
        error = np.linalg.norm(position - target)

        if error < 0.1:
            solid += 1
            if solid == 10:
                print(f'episode {episode_idx} step {i}: target={target} reached with position={position}')
                _, info = env.reset()
                episode_idx += 1
                if not info:  # No more episode
                    break
                else:  # Reset all counters before next episode starts
                    i = 0
                    solid = 0

        assert i < max_steps, f'max steps reached, position={position}, target={target}, error={error}'

    env.close()


if __name__ == '__main__':
    pass
