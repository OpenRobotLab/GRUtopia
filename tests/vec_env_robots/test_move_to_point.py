from collections import OrderedDict
from copy import deepcopy

import numpy as np

from internutopia.core.config import Config, SimConfig
from internutopia.core.util import has_display
from internutopia.core.vec_env import Env
from internutopia.macros import gm
from internutopia_extension import import_extensions
from internutopia_extension.configs.tasks import SingleInferenceTaskCfg

headless = not has_display()


def run(robot_0_cfg: tuple, robot_1_cfg: tuple, max_steps=5000):
    robot_0, action_0, target_0 = robot_0_cfg
    robot_1, action_1, target_1 = robot_1_cfg
    targets = [target_0, target_1]
    config = Config(
        simulator=SimConfig(
            physics_dt=1 / 240, rendering_dt=1 / 240, use_fabric=True, headless=headless, native=headless
        ),
        env_num=2,
        env_offset_size=10,
        task_configs=[
            SingleInferenceTaskCfg(
                scene_asset_path=gm.ASSET_PATH + '/scenes/empty.usd',
                robots=[robot_0.update(), robot_1.update()],
            )
            for _ in range(4)
        ],
    )

    import_extensions()

    env = Env(config)
    obs, _ = env.reset()

    i = [0, 0]  # for each env
    reset_idx = [0, 0]  # for each env

    env_action = [
        OrderedDict({robot_0.name: action_0, robot_1.name: action_1}),  # env0
        OrderedDict({robot_0.name: action_0, robot_1.name: action_1}),  # env1
    ]

    print(f'actions: {env_action}')
    solid = [[0, 0], [0, 0]]  # for each robot
    solid_reached = [[False, False], [False, False]]  # for each robot

    def has_reached(position, target) -> bool:
        error = np.linalg.norm(position - target)
        if error < 0.1:
            return True
        return False

    while env.simulation_app.is_running():
        env_obs, _, _, _, _ = env.step(action=env_action)
        if not any(env_obs):
            print('no more observations, exit')
            break
        for env_id, obs in enumerate(env_obs):
            i[env_id] += 1
            if not obs:
                continue

            assert (
                i[env_id] < max_steps
            ), f'env {env_id} reset_idx {reset_idx[env_id]}: max steps reached, env_obs={env_obs}'

            for robot_idx, robot_name in enumerate([robot_0.name, robot_1.name]):
                position = deepcopy(obs[robot_name]['position'])
                position[2] = targets[robot_idx][2]  # Make sure we ignore z axis

                reached = has_reached(position, targets[robot_idx])
                if reached:
                    solid[env_id][robot_idx] += 1
                if solid[env_id][robot_idx] == 10:
                    print(
                        f'env {env_id} reset_idx {reset_idx[env_id]} step {i[env_id]} robot {robot_name}: target={targets[robot_idx]} reached with position={position}'
                    )
                    solid_reached[env_id][robot_idx] = True

                if all(solid_reached[env_id]):
                    print(
                        f'env {env_id} reset_idx {reset_idx[env_id]} step {i[env_id]} : all robots have reached targets={targets}'
                    )
                    print(f'reset env {env_id}')
                    env.reset(env_ids=[env_id])
                    reset_idx[env_id] += 1
                    # Reset all counters before next episode starts
                    i[env_id] = 0
                    solid[env_id] = [0, 0]
                    solid_reached[env_id] = [False, False]

    env.close()


if __name__ == '__main__':
    pass
