def main():
    from internutopia.core.config import Config, SimConfig
    from internutopia.core.util import has_display
    from internutopia.core.vec_env import Env
    from internutopia.macros import gm
    from internutopia_extension import import_extensions
    from internutopia_extension.configs.robots.franka import (
        FrankaRobotCfg,
        arm_ik_cfg,
        gripper_cfg,
    )
    from internutopia_extension.configs.tasks import ManipulationTaskCfg

    headless = False
    if not has_display():
        headless = True

    franka = FrankaRobotCfg(
        position=(0, 0, 0),
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
        env_num=2,
        env_offset_size=4,
        task_configs=[
            ManipulationTaskCfg(
                scene_asset_path=gm.ASSET_PATH + '/scenes/empty.usd',
                robots=[franka.update()],
                prompt='Prompt test 1',
                target='franka_manipulation',
                episode_idx=0,
                max_steps=500,
            )
            for _ in range(8)
        ],
    )

    import_extensions()

    env = Env(config)
    import numpy as np
    from omni.isaac.core.utils.rotations import euler_angles_to_quat

    obs, _ = env.reset()
    task_name = list(env.runner.current_tasks.keys())[1]
    _robot = env.runner.current_tasks[task_name].robots['franka']

    actions = [
        {arm_ik_cfg.name: [np.array([0.4, 0, 0.45]), euler_angles_to_quat((0.0, 0.0, 0.0))]},
        {gripper_cfg.name: ['open']},
        {arm_ik_cfg.name: [np.array([0.4, 0.4, 0.1]), euler_angles_to_quat((np.pi / 2, np.pi / 2, np.pi / 2))]},
        {gripper_cfg.name: ['close']},
    ]

    i = 0
    _init_gripper_pos = None
    _350_gripper_pos = None

    no_more_episode = False
    while env.simulation_app.is_running():
        i += 1
        if i % 400 == 0:
            print(i)
            env_actions = actions[0]
        elif i % 400 == 100:
            print(i)
            env_actions = actions[1]
        elif i % 400 == 200:
            print(i)
            env_actions = actions[2]
        elif i % 400 == 300:
            print(i)
            env_actions = actions[3]
        else:
            env_actions = {}

        obs, _, terminated, _, _ = env.step(action=[{'franka': env_actions}, {'franka': env_actions}])

        if all(terminated) and no_more_episode:
            break

        if i == 250:
            _init_gripper_pos = obs[1]['franka']['controllers']['gripper_controller']['gripper_pos']

        if i == 350:
            assert np.linalg.norm(_robot.articulation.get_world_pose()[0] - _robot.articulation.get_pose()[0]) == 4
            assert np.linalg.norm(_robot.articulation.get_world_pose()[0] - obs[1]['franka']['position']) == 4
            _350_gripper_pos = obs[1]['franka']['controllers']['gripper_controller']['gripper_pos']
            _o, _ = env.reset([0])

        if i == 351:
            assert np.allclose(
                obs[1]['franka']['controllers']['gripper_controller']['gripper_pos'], _350_gripper_pos, atol=0.01
            )
            assert not np.allclose(
                obs[1]['franka']['controllers']['gripper_controller']['gripper_pos'], _init_gripper_pos, atol=0.01
            )

        reset_list = [idx for idx, t in enumerate(terminated) if t]
        if reset_list and not no_more_episode:
            _, info = env.reset(env_ids=reset_list)
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
