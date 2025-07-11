def main():
    import numpy as np

    from grutopia.core.config import Config, SimConfig
    from grutopia.core.util import has_display
    from grutopia.core.vec_env import Env
    from grutopia.macros import gm
    from grutopia_extension import import_extensions
    from grutopia_extension.configs.robots.franka import (
        FrankaRobotCfg,
        arm_ik_cfg,
        gripper_cfg,
    )
    from grutopia_extension.configs.tasks import (
        SingleInferenceEpisodeCfg,
        SingleInferenceTaskCfg,
    )

    headless = False
    if not has_display():
        headless = True

    franka_0 = FrankaRobotCfg(
        name='franka_0',
        prim_path='/franka_0',
        position=(0, 0, 0),
        controllers=[
            arm_ik_cfg,
            gripper_cfg,
        ],
    )
    franka_1 = FrankaRobotCfg(
        name='franka_1',
        prim_path='/franka_1',
        position=(3.0, 0, 0),
        controllers=[
            arm_ik_cfg,
            gripper_cfg,
        ],
    )

    config = Config(
        simulator=SimConfig(
            physics_dt=1 / 240,
            rendering_dt=1 / 240,
            use_fabric=False,
            rendering_interval=0,
            headless=headless,
            native=headless,
        ),
        task_config=SingleInferenceTaskCfg(
            env_num=2,
            offset_size=10,
            episodes=[
                SingleInferenceEpisodeCfg(
                    scene_asset_path=gm.ASSET_PATH + '/scenes/empty.usd',
                    robots=[franka_0.update(), franka_1.update()],
                )
                for _ in range(6)
            ],
        ),
    )

    import_extensions()

    env = Env(config)
    from omni.isaac.core.utils.rotations import euler_angles_to_quat

    obs, _ = env.reset()
    max_steps = 2000

    i = [0, 0]  # for each env
    reset_idx = [0, 0]  # for each env
    solid = [[0, 0], [0, 0]]  # for each robot
    solid_finished = [[False, False], [False, False]]  # for each robot

    env_action = [
        {
            franka_0.name: {
                arm_ik_cfg.name: [np.array([0.4, 0, 0.45]), euler_angles_to_quat((0.0, 0.0, 0.0))],
                gripper_cfg.name: ['open'],
            },
            franka_1.name: {
                arm_ik_cfg.name: [np.array([3.4, 0, 0.45]), euler_angles_to_quat((0.0, 0.0, 0.0))],
                gripper_cfg.name: ['close'],
            },
        },
        {
            franka_0.name: {
                arm_ik_cfg.name: [np.array([0.4, 0, 0.45]), euler_angles_to_quat((0.0, 0.0, 0.0))],
                gripper_cfg.name: ['open'],
            },
            franka_1.name: {
                arm_ik_cfg.name: [np.array([3.4, 0, 0.45]), euler_angles_to_quat((0.0, 0.0, 0.0))],
                gripper_cfg.name: ['close'],
            },
        },
    ]

    while env.simulation_app.is_running():
        env_obs, _, _, _, _ = env.step(action=env_action)
        if not any(env_obs):
            print('no more observations, exit')
            break
        for env_id, obs in enumerate(env_obs):
            i[env_id] += 1
            if not obs:
                continue

            assert (
                i[env_id] < max_steps
            ), f'env {env_id} reset_idx {reset_idx[env_id]}: max steps reached, env_obs ={env_obs}'
            for robot_idx, robot_name in enumerate([franka_0.name, franka_1.name]):
                finished = obs[robot_name]['controllers'][arm_ik_cfg.name]['finished']

                if finished:
                    solid[env_id][robot_idx] += 1
                if solid[env_id][robot_idx] == 100:
                    print(
                        f'env {env_id} reset_idx {reset_idx[env_id]} step {i[env_id]} robot {robot_name}: action finished, obs={obs}'
                    )
                    solid_finished[env_id][robot_idx] = True

                if all(solid_finished[env_id]):
                    print(f'env {env_id} reset_idx {reset_idx[env_id]} step {i[env_id]} : all robots have finished')
                    print(f'reset env {env_id}')
                    env.reset(env_ids=[env_id])
                    reset_idx[env_id] += 1
                    # Reset all counters before next episode starts
                    i[env_id] = 0
                    solid[env_id] = [0, 0]
                    solid_finished[env_id] = [False, False]

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
