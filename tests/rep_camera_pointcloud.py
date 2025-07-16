def main():
    import json
    import time

    import numpy as np

    from grutopia.core.config import Config, SimConfig
    from grutopia.core.gym_env import Env
    from grutopia.core.util import has_display
    from grutopia.macros import gm
    from grutopia_extension import import_extensions
    from grutopia_extension.configs.objects import DynamicCubeCfg
    from grutopia_extension.configs.robots.h1 import (
        H1RobotCfg,
        h1_camera_cfg,
        move_by_speed_cfg,
    )
    from grutopia_extension.configs.tasks import SingleInferenceTaskCfg

    t0 = time.perf_counter()

    headless = False
    if not has_display():
        headless = True

    h1 = H1RobotCfg(
        position=(0.0, 0.0, 1.05),
        controllers=[
            move_by_speed_cfg,
        ],
        sensors=[
            h1_camera_cfg.update(name='camera', resolution=(320, 240), enable=True, pointcloud=True),
        ],
    )

    config = Config(
        simulator=SimConfig(
            physics_dt=1 / 240, rendering_dt=1 / 240, use_fabric=True, headless=headless, native=headless
        ),
        task_configs=[
            SingleInferenceTaskCfg(
                scene_asset_path=gm.ASSET_PATH + '/scenes/empty.usd',
                scene_scale=(0.01, 0.01, 0.01),
                robots=[h1],
                objects=[
                    DynamicCubeCfg(
                        name='target_cube',
                        prim_path='/World/target_cube',
                        position=(2.0, 0.0, 0.5),
                        scale=(0.4, 0.4, 1.0),
                        color=(0.8, 0.0, 0.8),
                    )
                ],
            ),
        ],
    )

    print(config.model_dump_json(indent=4))

    import_extensions()

    t1 = time.perf_counter()
    env = Env(config)
    t2 = time.perf_counter()
    obs, _ = env.reset()
    t3 = time.perf_counter()
    print(f'========INIT OBS{obs}=============')

    i = 0

    move_action = {move_by_speed_cfg.name: [0.0, 0.0, 0.0]}

    def assert_pointcloud(pointcloud: np.ndarray):  # pointcloud shape is (N, 3)
        print(f'pointcloud shape: {pointcloud.shape}')
        error = 0.02
        cube_height = 1.0
        cube_x_min = 1.8
        cube_x_max = 2.2
        cube_y_min = -0.2
        cube_y_max = 0.2
        # Find all points within 0.8 < x < 1.2 and -0.2 < y < 0.2 (area of the target cube) with error tolerance.
        cube_x_mask = (pointcloud[:, 0] > (cube_x_min + error)) & (pointcloud[:, 0] < (cube_x_max - error))
        cube_y_mask = (pointcloud[:, 1] > (cube_y_min + error)) & (pointcloud[:, 1] < (cube_y_max - error))
        mask = cube_x_mask & cube_y_mask
        cube_points = pointcloud[mask]
        print(f'cube points shape: {cube_points.shape}')
        z_values = cube_points[:, 2]
        # Assert z values of all points are close to the cube height.
        cond = (z_values > (cube_height - error)) & (z_values < (cube_height + error))
        assert np.all(cond), f'unexpected pointcloud data within cube: {cube_points[~cond]}'

        # Find all points within out of the cube with error tolerance (~mask cannot be used because of cube edge error).
        floor_x_mask = (pointcloud[:, 0] < (cube_x_min - error)) | (pointcloud[:, 0] > (cube_x_max + error))
        floor_y_mask = (pointcloud[:, 1] < (cube_y_min - error)) | (pointcloud[:, 1] > (cube_y_max + error))
        # Limit the floor range (pointcloud numerical error get greater with further distance).
        range_x_mask = (pointcloud[:, 0] > -8.0) & (pointcloud[:, 0] < 8.0)
        range_y_mask = (pointcloud[:, 1] > -8.0) & (pointcloud[:, 1] < 8.0)
        mask = floor_x_mask & floor_y_mask & range_x_mask & range_y_mask
        floor_points = pointcloud[mask]
        print(f'floor points shape: {floor_points.shape}')
        z_values = floor_points[:, 2]
        # Assert z values of all points are close to the floor height (zero).
        cond = (z_values > -error) & (z_values < error)
        assert np.all(cond), f'unexpected pointcloud data within floor: {floor_points[~cond]}'

    while env.simulation_app.is_running():
        i += 1
        action = move_action
        obs, _, terminated, _, _ = env.step(action=action)
        if i % 500 == 0:
            print(i)
            pointcloud = obs['sensors'][h1_camera_cfg.name]['pointcloud']
            assert_pointcloud(pointcloud)
        if i == 2000:
            t4 = time.perf_counter()
            run_result = {
                'import_ext': t1 - t0,
                'create_env': t2 - t1,
                'reset_env': t3 - t2,
                '2k_step': t4 - t3,
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
