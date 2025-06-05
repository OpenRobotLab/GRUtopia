def main():
    from grutopia.core.config import Config, SimConfig
    from grutopia.core.runtime import SimulatorRuntime
    from grutopia.core.util import has_display
    from grutopia.core.vec_env import Env
    from grutopia.macros import gm
    from grutopia_extension import import_extensions
    from grutopia_extension.configs.robots.h1 import (
        H1RobotCfg,
        h1_camera_cfg,
        h1_tp_camera_cfg,
        move_by_speed_cfg,
        rotate_cfg,
    )
    from grutopia_extension.configs.tasks import (
        SingleInferenceEpisodeCfg,
        SingleInferenceTaskCfg,
    )

    headless = False
    if not has_display():
        headless = True

    h1 = H1RobotCfg(
        position=(0.0, 0.0, 1.05),
        controllers=[
            move_by_speed_cfg,
            rotate_cfg,
        ],
        sensors=[
            h1_camera_cfg.update(name='camera', resolution=(320, 240), enable=False),
            h1_tp_camera_cfg.update(enable=False),
        ],
    )

    config = Config(
        simulator=SimConfig(physics_dt=1 / 240, rendering_dt=1 / 240, use_fabric=True, rendering_interval=20),
        task_config=SingleInferenceTaskCfg(
            env_num=2,
            offset_size=10,
            episodes=[
                SingleInferenceEpisodeCfg(
                    scene_asset_path=gm.ASSET_PATH + '/scenes/empty.usd',
                    scene_scale=(0.01, 0.01, 0.01),
                    robots=[h1.update()],
                ),
                SingleInferenceEpisodeCfg(
                    scene_asset_path=gm.ASSET_PATH + '/scenes/empty.usd',
                    scene_scale=(0.01, 0.01, 0.01),
                    robots=[h1.update()],
                ),
            ],
        ),
    )

    print(config.model_dump_json(indent=4))

    sim_runtime = SimulatorRuntime(config_class=config, headless=headless, native=headless)

    import_extensions()

    env = Env(sim_runtime)
    obs, _ = env.reset()

    i = 0
    move_action = {move_by_speed_cfg.name: [1, 0, 0]}

    while env.simulation_app.is_running():
        i += 1
        obs, reward, terminated, _, _ = env.step(action=[{'h1': move_action}, {'h1': move_action}])
        if i % 100 == 0:
            print(i)

        if i % 500 == 0:
            _, info = env.reset(env_ids=[0])
            assert len(info) == 0

        if i == 10000:
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
