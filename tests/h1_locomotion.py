def main():
    import json
    import time

    from grutopia.core.config import Config, SimConfig
    from grutopia.core.gym_env import Env
    from grutopia.core.runtime import SimulatorRuntime
    from grutopia.core.util import has_display
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
        SingleInferenceEpisodeCfg,
        SingleInferenceTaskCfg,
    )

    t0 = time.perf_counter()

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
            h1_camera_cfg.update(name='camera', resolution=(320, 240), enable=False),
            h1_tp_camera_cfg.update(enable=False),
        ],
    )

    config = Config(
        simulator=SimConfig(physics_dt=1 / 240, rendering_dt=1 / 240, use_fabric=True),
        task_config=SingleInferenceTaskCfg(
            episodes=[
                SingleInferenceEpisodeCfg(
                    scene_asset_path=gm.ASSET_PATH + '/scenes/empty.usd',
                    scene_scale=(0.01, 0.01, 0.01),
                    robots=[h1_1],
                ),
            ],
        ),
    )

    print(config.model_dump_json(indent=4))

    sim_runtime = SimulatorRuntime(config_class=config, headless=headless, native=headless)

    t1 = time.perf_counter()
    import_extensions()
    # import custom extensions here.

    t2 = time.perf_counter()
    env = Env(sim_runtime)
    t3 = time.perf_counter()
    obs, _ = env.reset()
    t4 = time.perf_counter()
    print(f'========INIT OBS{obs}=============')

    path = [(1.0, 0.0, 0.0), (1.0, 1.0, 0.0), (3.0, 4.0, 0.0)]
    i = 0

    move_action = {move_along_path_cfg.name: [path]}

    while env.simulation_app.is_running():
        i += 1
        action = move_action
        obs, _, terminated, _, _ = env.step(action=action)
        if i % 100 == 0:
            print(i)
            print(obs)
            pos = obs['position']
            assert pos[0] < 5.0 and pos[1] < 5.0 and pos[2] < 2.0, 'out of range'
        if i == 2000:
            t5 = time.perf_counter()
            run_result = {
                'start_sim_app': t1 - t0,
                'import_ext': t2 - t1,
                'create_env': t3 - t2,
                'reset_env': t4 - t3,
                '2k_step': t5 - t4,
            }
            with open('./test_result.json', 'w', encoding='utf-8') as f:
                json.dump(run_result, f, ensure_ascii=False, indent=4)
            print(f'times: {run_result}')
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
