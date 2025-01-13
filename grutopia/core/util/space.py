import gymnasium as gym
import numpy as np


# TODO get action space based on the specific task, currently the hardcoded value will be returned.
def get_action_space_by_task(task_type: str) -> gym.Space:
    return gym.spaces.Dict(
        {
            'move_along_path': gym.spaces.Sequence(
                gym.spaces.Tuple(
                    (
                        gym.spaces.Box(low=-np.inf, high=np.inf, shape=(), dtype=np.float32),
                        gym.spaces.Box(low=-np.inf, high=np.inf, shape=(), dtype=np.float32),
                        gym.spaces.Box(low=-np.inf, high=np.inf, shape=(), dtype=np.float32),
                    )
                )
            )
        }
    )


# TODO get observation space based on the specific task, currently the hardcoded value will be returned.
def get_observation_space_by_task(task_type: str) -> gym.Space:
    return gym.spaces.Dict(
        {
            'position': gym.spaces.Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32),
            'orientation': gym.spaces.Box(low=-np.inf, high=np.inf, shape=(4,), dtype=np.float32),
        }
    )
