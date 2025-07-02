def main():
    from grutopia.core.config import Config, SimConfig
    from grutopia.core.runtime import SimulatorRuntime
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
        ManipulationEpisodeCfg,
        ManipulationExtra,
        ManipulationTaskCfg,
        ManipulationTaskSetting,
    )

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
        simulator=SimConfig(physics_dt=1 / 240, rendering_dt=1 / 240, use_fabric=False, rendering_interval=0),
        task_config=ManipulationTaskCfg(
            env_num=2,
            offset_size=4,
            episodes=[
                ManipulationEpisodeCfg(
                    scene_asset_path=gm.ASSET_PATH + '/scenes/empty.usd',
                    robots=[franka.update()],
                    extra=ManipulationExtra(
                        prompt='Prompt test 1',
                        target='franka_manipulation',
                        episode_idx=0,
                    ),
                )
                for _ in range(6)
            ],
            task_settings=ManipulationTaskSetting(max_step=500),
        ),
    )
    sim_runtime = SimulatorRuntime(config_class=config, headless=headless, native=headless)
    import_extensions()
    import numpy as np
    from omni.isaac.core.utils.rotations import euler_angles_to_quat

    env = Env(sim_runtime)
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
        if i == 350:
            assert np.linalg.norm(_robot.isaac_robot.get_world_pose()[0] - _robot.isaac_robot.get_pose()[0]) == 4
            assert np.linalg.norm(_robot.isaac_robot.get_world_pose()[0] - obs[1]['franka']['position']) == 4
        reset_list = [idx for idx, t in enumerate(terminated) if t]
        if reset_list:
            _, info = env.reset(env_ids=reset_list)
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
