def main():
    import numpy as np

    from internutopia.core.config import Config, SimConfig
    from internutopia.core.util import has_display
    from internutopia.core.vec_env import Env
    from internutopia.macros import gm
    from internutopia_extension import import_extensions
    from internutopia_extension.configs.robots.h1 import (
        H1RobotCfg,
        move_by_speed_cfg,
        rotate_cfg,
    )
    from internutopia_extension.configs.tasks import SingleInferenceTaskCfg

    headless = False
    if not has_display():
        headless = True

    h1 = H1RobotCfg(
        position=(0.0, 0.0, 1.05),
        controllers=[
            move_by_speed_cfg,
            rotate_cfg,
        ],
        sensors=[],
    )

    config = Config(
        simulator=SimConfig(
            physics_dt=1 / 240, rendering_dt=1 / 240, use_fabric=True, rendering_interval=20, headless=headless
        ),
        env_num=2,
        env_offset_size=10,
        task_configs=[
            SingleInferenceTaskCfg(
                scene_asset_path=gm.ASSET_PATH + '/scenes/empty.usd',
                scene_scale=(0.01, 0.01, 0.01),
                robots=[h1],
            ),
            SingleInferenceTaskCfg(
                scene_asset_path=gm.ASSET_PATH + '/scenes/empty.usd',
                scene_scale=(0.01, 0.01, 0.01),
                robots=[h1],
            ),
        ],
    )

    print(config.model_dump_json(indent=4))

    import_extensions()

    env = Env(config)
    obs, _ = env.reset()

    i = 0
    move_action = {move_by_speed_cfg.name: [1, 0, 0]}
    _pos = [0, 0, 1.05]

    while env.simulation_app.is_running():
        i += 1
        obs, reward, terminated, _, _ = env.step(action=[{'h1_1': move_action}, {'h1': move_action}])
        if i == 1:
            _pos = obs[1]['h1']['position']

        if i % 100 == 0:
            print(i)

        if i % 500 == 0:
            _, info = env.reset(env_ids=[0])
            assert len(info) == 1
            assert None in info

        if i == 501:
            _pos2 = obs[1]['h1']['position']
            assert np.linalg.norm(_pos2 - _pos) > 1

        if i == 5000:
            break
    env.close()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f'exception is {e}')
        import sys
        import traceback

        traceback.print_exc()
        sys.exit(1)
