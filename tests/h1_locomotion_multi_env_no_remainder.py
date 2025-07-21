def main():
    from collections import OrderedDict

    from internutopia.core.config import Config, SimConfig
    from internutopia.core.util import has_display
    from internutopia.core.vec_env import Env
    from internutopia.macros import gm
    from internutopia_extension import import_extensions
    from internutopia_extension.configs.robots.h1 import (
        H1RobotCfg,
        h1_camera_cfg,
        h1_tp_camera_cfg,
        move_along_path_cfg,
        move_by_speed_cfg,
        rotate_cfg,
    )
    from internutopia_extension.configs.tasks import FiniteStepTaskCfg

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
        simulator=SimConfig(
            physics_dt=1 / 240,
            rendering_dt=1 / 240,
            use_fabric=True,
            rendering_interval=20,
            headless=headless,
            native=headless,
        ),
        env_num=4,
        env_offset_size=10,
        task_configs=[
            FiniteStepTaskCfg(
                max_steps=500,
                scene_asset_path=gm.ASSET_PATH + '/scenes/empty.usd',
                scene_scale=(0.01, 0.01, 0.01),
                robots=[h1_1.update(), h1.update()],
            )
            for _ in range(8)
        ],
    )

    print(config.model_dump_json(indent=4))

    import_extensions()

    env = Env(config)
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

        assert len(obs) == 4
        assert len(terminated_status) == 4

        if all(terminated_status) and no_more_episode:
            break

        if i % 100 == 0:
            print(i)

        if any(terminated_status) and not no_more_episode:
            obs, info = env.reset(env_ids=[idx for idx, term in enumerate(terminated_status) if term])
            assert len(obs) == 4
            assert len(info) == 4
            if i < 700:
                assert all(info)
            if None in info:
                assert not any(info)
                assert not any(obs)
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
