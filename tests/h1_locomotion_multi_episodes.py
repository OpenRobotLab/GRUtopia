def main():
    from grutopia.core.config import Config, SimConfig
    from grutopia.core.gym_env import Env
    from grutopia.core.runtime import SimulatorRuntime
    from grutopia.core.util import has_display
    from grutopia.macros import gm
    from grutopia_extension import import_extensions
    from grutopia_extension.configs.robots.h1 import (
        H1RobotCfg,
        h1_camera_cfg,
        move_along_path_cfg,
        move_by_speed_cfg,
        rotate_cfg,
    )
    from grutopia_extension.configs.tasks import (
        FiniteStepTaskCfg,
        FiniteStepTaskEpisodeCfg,
    )

    headless = False
    if not has_display():
        headless = True

    h1_1 = H1RobotCfg(
        position=(0.0, 0.0, 1.05),
        controllers=[
            move_by_speed_cfg,
            move_along_path_cfg,
            rotate_cfg,
        ],
        sensors=[
            h1_camera_cfg.update(name='camera', resolution=(64, 64), enable=True),
        ],
    )

    config = Config(
        simulator=SimConfig(physics_dt=1 / 240, rendering_dt=1 / 240, use_fabric=True, rendering_interval=20),
        task_config=FiniteStepTaskCfg(
            task_settings={'max_step': 501},
            episodes=[
                FiniteStepTaskEpisodeCfg(
                    scene_asset_path=gm.ASSET_PATH + '/scenes/empty.usd',
                    scene_scale=(0.01, 0.01, 0.01),
                    robots=[h1_1.update()],
                ),
                FiniteStepTaskEpisodeCfg(
                    scene_asset_path=gm.ASSET_PATH + '/scenes/empty.usd',
                    scene_scale=(0.01, 0.01, 0.01),
                    robots=[h1_1.update()],
                ),
            ],
        ),
    )

    print(config.model_dump_json(indent=4))
    sim_runtime = SimulatorRuntime(config_class=config, headless=headless, native=headless)
    import_extensions()
    env = Env(sim_runtime)
    obs, _ = env.reset()

    env.warm_up(5, physics=False)

    path = [(1.0, 0.0, 0.0), (1.0, 1.0, 0.0), (3.0, 4.0, 0.0)]
    i = 0
    move_action = {move_along_path_cfg.name: [path]}
    while env.simulation_app.is_running():
        i += 1
        obs, _, terminated, _, _ = env.step(action=move_action)

        if i == 1 or i == 101:
            assert obs['sensors']['camera']['rgba'].shape == (64, 64, 4)

        if i % 100 == 0:
            print(i)
        if terminated:
            _, info = env.reset()

            env.warm_up(5, physics=False)

            if not info:
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
