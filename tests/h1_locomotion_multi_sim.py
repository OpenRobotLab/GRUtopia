def main():
    from multi_sim_task import (  # noqa F401
        MultiSimTestEpisodeCfg,
        MultiSimTestH1Robot,
        MultiSimTestH1RobotCfg,
        MultiSimTestTask,
        MultiSimTestTaskCfg,
    )

    from grutopia.core.config import Config, SimConfig
    from grutopia.core.config.distribution import RayDistributionCfg
    from grutopia.core.vec_env import Env
    from grutopia.macros import gm

    headless = True
    num_episode = 9
    num_step = 50
    h1 = MultiSimTestH1RobotCfg()

    episodes = [
        MultiSimTestEpisodeCfg(
            scene_asset_path=gm.ASSET_PATH + '/scenes/empty.usd',
            scene_scale=(0.01, 0.01, 0.01),
            robots=[h1.update()],
            extra={
                'episode_id': episode_id,
            },
        )
        for episode_id in range(num_episode)
    ]

    config = Config(
        simulator=SimConfig(physics_dt=1 / 240, rendering_dt=1 / 240, use_fabric=False, headless=headless),
        task_config=MultiSimTestTaskCfg(
            env_num=2,
            episodes=episodes,
        ),
    ).distribute(
        RayDistributionCfg(
            proc_num=2,
            gpu_num_per_proc=0.5,
        )
    )

    from grutopia_extension import import_extensions

    import_extensions()

    env = Env(config)
    obs, task_runtimes = env.reset()
    env.warm_up(5, physics=False)

    assert env.is_render is None, 'need be None'
    try:
        env.runner
        assert False, 'need raise NotImplementedError'
    except NotImplementedError:
        pass
    active_runtimes = env.active_runtimes
    assert len(active_runtimes) == 4
    for env_id, task_runtime in enumerate(task_runtimes):
        assert active_runtimes[env_id] == task_runtime
    try:
        env.get_dt()
        assert False, 'need raise NotImplementedError'
    except NotImplementedError:
        pass
    try:
        env.simulation_app
        assert False, 'need raise NotImplementedError'
    except NotImplementedError:
        pass
    try:
        env.finished()
        assert False, 'need raise NotImplementedError'
    except NotImplementedError:
        pass

    all_episode = []
    active_episode_ids = [obs[env_id]['h1']['episode_id'] for env_id in range(len(obs))]
    all_episode.extend(active_episode_ids)
    action = [{'h1': {'test_controller': episode_id}} for episode_id in active_episode_ids]

    assert len(env.get_observations()) == len(active_episode_ids), 'wrong obs length'

    i = 0
    no_more_episode = False
    while True:
        i += 1
        if i % 100 == 0:
            print(i)

        obs, _, terminated_status, _, _ = env.step(action=action)

        if all(terminated_status) and no_more_episode:
            break

        if any(terminated_status) and not no_more_episode:
            assert len(obs) == len(active_episode_ids), 'len(obs) should equal to len(active_episode_ids)'
            for env_id, env_obs in enumerate(obs):
                if not env_obs:
                    continue
                episode_id = active_episode_ids[env_id]
                observed_actions = env_obs['h1']['observed_actions']
                assert len(observed_actions) == num_step, f'len(observed_actions) should be {num_step}'
                assert all(
                    action == episode_id for action in observed_actions
                ), 'env get wrong action'  # ensure each action is correct

            # all episodes ends in the same step
            assert all(terminated_status), 'all episodes should end in the same step'
            obs, info = env.reset()
            if None in info:
                no_more_episode = True
            active_episode_ids = [env_obs['h1']['episode_id'] if env_obs else None for env_obs in obs]
            all_episode.extend([episode_id for episode_id in active_episode_ids if episode_id is not None])
            action = [{'h1': {'test_controller': episode_id}} for episode_id in active_episode_ids]

    assert len(all_episode) == num_episode, 'duplicated episode'
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
