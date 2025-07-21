from copy import deepcopy
from typing import Any, Dict, Tuple

import numpy as np

from internutopia.core.config import Config, RobotCfg, SimConfig
from internutopia.core.gym_env import Env
from internutopia.core.util import has_display
from internutopia.macros import gm
from internutopia_extension import import_extensions
from internutopia_extension.configs.tasks import SingleInferenceTaskCfg

headless = not has_display()


def run(
    robot: RobotCfg, action: Dict[str, Any], target: Tuple[float, float, float, float], threshold=0.1, max_steps=2000
):
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

    def get_abs_delta_z_rot(x, y):
        from omni.isaac.core.utils.rotations import quat_to_euler_angles

        delta_z_rot = quat_to_euler_angles(x)[-1] - quat_to_euler_angles(y)[-1]
        delta_z_rot = delta_z_rot % (2 * np.pi)
        if delta_z_rot > np.pi:
            delta_z_rot = delta_z_rot - 2 * np.pi

        return abs(delta_z_rot)

    while env.simulation_app.is_running():
        i += 1
        obs, _, _, _, _ = env.step(action=env_action)

        orientation = deepcopy(obs['orientation'])
        orientation[2] = target[2]  # Make sure we ignore z axis
        error = get_abs_delta_z_rot(orientation, target)

        if error < threshold:
            solid += 1
            if solid == 100:
                print(
                    f'episode {episode_idx} step {i}: target={target} reached with target={orientation}, error={error}'
                )
                _, info = env.reset()
                episode_idx += 1
                if not info:  # No more episode
                    break
                else:  # Reset all counters before next episode starts
                    i = 0
                    solid = 0

        assert i < max_steps, f'max steps reached, orientation={orientation}, target={target}, z axis error={error}'

    env.close()


if __name__ == '__main__':
    pass
