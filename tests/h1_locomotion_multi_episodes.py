def main():
    from internutopia.core.config import Config, SimConfig
    from internutopia.core.gym_env import Env
    from internutopia.core.util import has_display
    from internutopia.macros import gm
    from internutopia_extension import import_extensions
    from internutopia_extension.configs.robots.h1 import (
        H1RobotCfg,
        h1_camera_cfg,
        move_along_path_cfg,
        move_by_speed_cfg,
        rotate_cfg,
    )
    from internutopia_extension.configs.tasks import FiniteStepTaskCfg

    headless = False
    if not has_display():
        headless = True

    h1 = H1RobotCfg(
        position=(0.0, 0.0, 1.05),
        controllers=[
            move_by_speed_cfg,
            move_along_path_cfg,
            rotate_cfg,
        ],
        sensors=[
            h1_camera_cfg.update(name='camera', resolution=(64, 64)),
        ],
    )

    h1_1 = h1.update(
        sensors=[
            h1_camera_cfg.update(name='camera_1', resolution=(64, 64)),
        ]
    )
    h1_2 = h1.update(
        sensors=[
            h1_camera_cfg.update(name='camera_2', resolution=(64, 64)),
        ]
    )
    h1_3 = h1.update(
        sensors=[
            h1_camera_cfg.update(name='camera_3', resolution=(64, 64)),
        ]
    )
    h1_4 = h1.update(
        sensors=[
            h1_camera_cfg.update(name='camera_4', resolution=(64, 64)),
        ]
    )

    config = Config(
        simulator=SimConfig(
            physics_dt=1 / 240, rendering_dt=1 / 240, use_fabric=True, headless=headless, native=headless
        ),
        task_configs=[
            FiniteStepTaskCfg(
                max_steps=100,
                scene_asset_path=gm.ASSET_PATH + '/scenes/empty.usd',
                scene_scale=(0.01, 0.01, 0.01),
                robots=[h1],
            ),
            FiniteStepTaskCfg(
                max_steps=100,
                scene_asset_path=gm.ASSET_PATH + '/scenes/empty.usd',
                scene_scale=(0.01, 0.01, 0.01),
                robots=[h1_1],
            ),
            FiniteStepTaskCfg(
                max_steps=100,
                scene_asset_path=gm.ASSET_PATH + '/scenes/empty.usd',
                scene_scale=(0.01, 0.01, 0.01),
                robots=[h1_2],
            ),
            FiniteStepTaskCfg(
                max_steps=100,
                scene_asset_path=gm.ASSET_PATH + '/scenes/empty.usd',
                scene_scale=(0.01, 0.01, 0.01),
                robots=[h1_3],
            ),
            FiniteStepTaskCfg(
                max_steps=100,
                scene_asset_path=gm.ASSET_PATH + '/scenes/empty.usd',
                scene_scale=(0.01, 0.01, 0.01),
                robots=[h1_4],
            ),
        ],
    )

    print(config.model_dump_json(indent=4))

    import_extensions()

    env = Env(config)
    obs, _ = env.reset()

    env.warm_up(5, physics=False)

    path = [(1.0, 0.0, 0.0), (1.0, 1.0, 0.0), (3.0, 4.0, 0.0)]
    i = 0
    move_action = {move_along_path_cfg.name: [path]}
    while env.simulation_app.is_running():
        i += 1
        obs, _, terminated, _, _ = env.step(action=move_action)

        if i % 100 == 0:
            print(i)

        if i == 1:
            assert 'camera' in obs['sensors']
            assert obs['sensors']['camera']['rgba'].shape == (64, 64, 4)

        if i == 101:
            assert 'camera_1' in obs['sensors']
            assert obs['sensors']['camera_1']['rgba'].shape == (64, 64, 4)

        if i == 201:
            assert 'camera_2' in obs['sensors']

        if i == 301:
            assert 'camera_3' in obs['sensors']

        if i == 401:
            assert 'camera_4' in obs['sensors']

        if terminated:
            _, info = env.reset()

            env.warm_up(5, physics=False)

            if info is None:
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
