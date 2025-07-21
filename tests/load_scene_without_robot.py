def main():
    from internutopia.core.config import Config, SimConfig
    from internutopia.core.gym_env import Env
    from internutopia.core.util import has_display
    from internutopia.macros import gm
    from internutopia_extension import import_extensions
    from internutopia_extension.configs.tasks import SingleInferenceTaskCfg

    headless = False
    if not has_display():
        headless = True

    config = Config(
        simulator=SimConfig(
            physics_dt=1 / 240, rendering_dt=1 / 240, use_fabric=False, headless=headless, native=headless
        ),
        task_configs=[
            SingleInferenceTaskCfg(
                scene_asset_path=gm.ASSET_PATH + '/scenes/empty.usd',
                scene_scale=(0.01, 0.01, 0.01),
                robots=[],
            ),
        ],
    )

    print(config.model_dump_json(indent=4))

    import_extensions()

    env = Env(config)
    obs, _ = env.reset()
    print(f'========INIT OBS{obs}=============')

    i = 0

    while env.simulation_app.is_running():
        i += 1
        obs, _, terminated, _, _ = env.step(action={})

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
