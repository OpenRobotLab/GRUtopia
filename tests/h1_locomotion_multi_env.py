def main():
    from collections import OrderedDict

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

    h1 = H1RobotCfg(
        position=(2.0, 0.0, 1.05),
        controllers=[
            move_by_speed_cfg,
            move_along_path_cfg,
            rotate_cfg,
        ],
        sensors=[
            h1_camera_cfg.update(name='camera', resolution=(160, 160), enable=True, depth=False),
            h1_tp_camera_cfg.update(enable=False),
        ],
    )

    h1_1 = H1RobotCfg(
        name='h1_1',
        prim_path='/h1_1',
        position=(0.0, 0.0, 1.05),
        controllers=[
            move_by_speed_cfg,
            move_along_path_cfg,
            rotate_cfg,
        ],
        sensors=[
            h1_camera_cfg.update(name='camera', resolution=(320, 240), enable=False),
            h1_tp_camera_cfg.update(enable=False),
        ],
    )

    config = Config(
        simulator=SimConfig(physics_dt=1 / 240, rendering_dt=1 / 240, use_fabric=True, rendering_interval=20),
        task_config=FiniteStepTaskCfg(
            env_num=4,
            offset_size=10,
            task_settings={'max_step': 500},
            episodes=[
                FiniteStepTaskEpisodeCfg(
                    scene_asset_path=gm.ASSET_PATH + '/scenes/empty.usd',
                    scene_scale=(0.01, 0.01, 0.01),
                    robots=[h1_1.update(), h1.update()],
                )
                for _ in range(9)
            ],
        ),
    )

    print(config.model_dump_json(indent=4))

    sim_runtime = SimulatorRuntime(config_class=config, headless=headless, native=headless)

    import_extensions()
    # import custom extensions here.

    env = Env(sim_runtime)
    obs, _ = env.reset()
    assert len(obs) == 4

    i = 0

    move_action = {move_by_speed_cfg.name: [1, 0, 0]}
    no_more_episode = False

    while env.simulation_app.is_running():
        i += 1
        action_0 = OrderedDict({'h1': move_action, 'h1_1': move_action})
        action_1 = OrderedDict({'h1': move_action, 'h1_1': move_action})
        action_2 = OrderedDict({'h1': move_action, 'h1_1': move_action})
        action_3 = OrderedDict({'h1': move_action, 'h1_1': move_action})
        obs, _, terminated_status, _, _ = env.step(action=[action_0, action_1, action_2, action_3])

        if False not in terminated_status and no_more_episode:
            break

        if i % 100 == 0:
            print(i)

        if i == 1300:
            assert len(obs) == 4

        if True in terminated_status and not no_more_episode:
            obs, info = env.reset(env_ids=[idx for idx, term in enumerate(terminated_status) if term])
            assert len(obs) == 4
            if i < 700:
                assert None not in info
            if not info or None in info:
                assert None in info
                assert sum(1 for item in info if item is None) == 3
                no_more_episode = True

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
