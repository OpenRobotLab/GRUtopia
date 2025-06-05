def main():
    from grutopia.core.config import Config, SimConfig
    from grutopia.core.runtime import SimulatorRuntime
    from grutopia.core.util import has_display
    from grutopia.core.vec_env import Env
    from grutopia.macros import gm
    from grutopia_extension import import_extensions
    from grutopia_extension.configs.tasks import (
        SingleInferenceEpisodeCfg,
        SingleInferenceTaskCfg,
    )

    headless = False
    if not has_display():
        headless = True

    config = Config(
        simulator=SimConfig(physics_dt=1 / 240, rendering_dt=1 / 240, use_fabric=False),
        task_config=SingleInferenceTaskCfg(
            env_num=2,
            offset_size=10,
            episodes=[
                SingleInferenceEpisodeCfg(
                    scene_asset_path=gm.ASSET_PATH + '/scenes/empty.usd',
                    scene_scale=(0.01, 0.01, 0.01),
                    robots=[],
                ),
                SingleInferenceEpisodeCfg(
                    scene_asset_path=gm.ASSET_PATH + '/scenes/empty.usd',
                    scene_scale=(0.01, 0.01, 0.01),
                    robots=[],
                ),
            ],
        ),
    )

    print(config.model_dump_json(indent=4))

    sim_runtime = SimulatorRuntime(config_class=config, headless=headless, native=headless)

    import_extensions()

    env = Env(sim_runtime)
    obs, _ = env.reset()
    print(f'========INIT OBS{obs}=============')

    i = 0

    while env.simulation_app.is_running():
        i += 1
        obs, _, terminated, _, _ = env.step(action=[{}, {}])

        if i == 2000:
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
