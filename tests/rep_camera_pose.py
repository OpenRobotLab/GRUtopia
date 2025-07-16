def main():
    import numpy as np

    from grutopia.core.config import Config, SimConfig
    from grutopia.core.gym_env import Env
    from grutopia.core.util import has_display
    from grutopia.macros import gm
    from grutopia_extension import import_extensions
    from grutopia_extension.configs.robots.h1 import (
        H1RobotCfg,
        h1_tp_camera_cfg,
        move_by_speed_cfg,
    )
    from grutopia_extension.configs.tasks import SingleInferenceTaskCfg

    headless = False
    if not has_display():
        headless = True

    h1 = H1RobotCfg(
        position=(0.0, 0.0, 1.05),
        controllers=[
            move_by_speed_cfg,
        ],
        sensors=[
            h1_tp_camera_cfg.update(
                name='camera',
                resolution=(320, 240),
                enable=True,
                depth=True,
            ),
        ],
    )

    config = Config(
        simulator=SimConfig(
            physics_dt=1 / 240, rendering_dt=1 / 240, use_fabric=False, headless=headless, native=headless
        ),
        task_configs=[
            SingleInferenceTaskCfg(
                scene_asset_path=gm.ASSET_PATH + '/scenes/empty.usd',
                scene_scale=(0.01, 0.01, 0.01),
                robots=[h1],
                objects=[],
            ),
        ],
    )

    print(config.model_dump_json(indent=4))

    import_extensions()

    env = Env(config)
    obs, _ = env.reset()
    task_name = list(env.runner.current_tasks.keys())[0]
    camera = env.runner.current_tasks[task_name].robots['h1'].sensors['camera']
    print(f'========INIT OBS{obs}=============')

    i = 0

    move_action = {move_by_speed_cfg.name: [1, 0.0, 0.0]}

    while env.simulation_app.is_running():
        i += 1
        action = move_action
        obs, _, _, _, _ = env.step(action=action)
        robot_pose = obs['position']
        camera_pose = camera.get_world_pose()
        camera.set_world_pose(
            [robot_pose[0], robot_pose[1], robot_pose[2] + 5],
            np.array([-0.70710678, 0.0, 0.0, 0.70710678]),
        )

        if i % 100 == 0:
            distance = np.linalg.norm(robot_pose[:2] - camera_pose[0][:2])
            assert distance < 0.1, 'camera position did not follow the robot position'
            rgba = obs['sensors']['camera']['rgba']
            depth = obs['sensors']['camera']['depth']
            assert rgba.shape == (240, 320, 4), 'unexpected rgba shape'
            assert depth.shape == (240, 320), 'unexpected depth shape'
            assert len(obs['sensors']['camera'].keys()) == 2, 'unexpected keys in camera obs'
            print(i)
        if i == 500:
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
