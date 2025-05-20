from copy import deepcopy
from typing import Any, Dict, Tuple

from grutopia.core.config import Config, SimConfig
from grutopia.core.gym_env import Env
from grutopia.core.runtime import SimulatorRuntime
from grutopia.core.util import has_display
from grutopia.macros import gm
from grutopia_extension import import_extensions
from grutopia_extension.configs.tasks import (
    SingleInferenceEpisodeCfg,
    SingleInferenceTaskCfg,
)

headless = not has_display()


def run(robot, action: Dict[str, Any], target: Tuple[float, float, float], max_steps=5000):
    config = Config(
        simulator=SimConfig(physics_dt=1 / 240, rendering_dt=1 / 240, use_fabric=True),
        task_config=SingleInferenceTaskCfg(
            episodes=[
                SingleInferenceEpisodeCfg(
                    scene_asset_path=gm.ASSET_PATH + '/scenes/empty.usd',
                    robots=[robot],
                ),
            ],
        ),
    )

    sim_runtime = SimulatorRuntime(config_class=config, headless=headless, native=headless)

    import_extensions()
    # import custom extensions here.
    import numpy as np

    env = Env(sim_runtime)
    obs, _ = env.reset()

    i = 0

    env_action = action

    print(f'actions: {env_action}')
    confirm = 0

    while env.simulation_app.is_running():
        i += 1
        obs, _, _, _, _ = env.step(action=env_action)

        position = deepcopy(obs['position'])
        position[2] = target[2]  # Make sure we ignore z axis
        error = np.linalg.norm(position - target)

        if error < 0.1:
            confirm += 1
            if confirm == 10:
                print(f'step {i}: target={target} reached with position={position}')
                break
        assert i < max_steps, f'max steps reached, position={position}, target={target}, error={error}'

    env.close()


if __name__ == '__main__':
    pass
