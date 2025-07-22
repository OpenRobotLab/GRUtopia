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
        ),
        env_num=3,
        env_offset_size=5,
        task_configs=[
            FiniteStepTaskCfg(
                max_steps=500,
                scene_asset_path=gm.ASSET_PATH + '/scenes/empty.usd',
                scene_scale=(0.01, 0.01, 0.01),
                robots=[h1_1, h1],
            ),
            FiniteStepTaskCfg(
                max_steps=500,
                scene_asset_path=gm.ASSET_PATH + '/scenes/empty.usd',
                scene_scale=(0.01, 0.01, 0.01),
                robots=[h1_1, h1],
            ),
        ],
    )

    print(config.model_dump_json(indent=4))

    import_extensions()

    env = Env(config)
    _, task_configs = env.reset()
    assert len(task_configs) == 3
    assert task_configs[0] is not None and task_configs[1] is not None
    assert task_configs[2] is None

    i = 0

    move_action = {move_by_speed_cfg.name: [1, 0, 0]}
    no_more_episode = False

    while env.simulation_app.is_running():
        i += 1
        action = OrderedDict({'h1': move_action, 'h1_1': move_action})
        obs, _, terminated_status, _, _ = env.step(action=[action for _ in range(3)])
        assert len(obs) == 3
        assert obs[0] is not None and obs[1] is not None
        assert obs[2] is None

        if all(terminated_status) and no_more_episode:
            break

        if i % 100 == 0:
            print(i)

        if any(terminated_status) and not no_more_episode:
            env_ids = [idx for idx, term in enumerate(terminated_status) if term]
            _, info = env.reset(env_ids=env_ids)
            assert len(info) == len(env_ids)
            if None in info:
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
