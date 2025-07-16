from test_move_to_point import run

from grutopia_extension.configs.robots.g1 import G1RobotCfg, move_to_point_cfg

if __name__ == '__main__':
    try:
        target = (3.0, 2.0, 0.0)
        case = {
            'robot': G1RobotCfg(
                position=(0.0, 0.0, 0.8),
                controllers=[move_to_point_cfg],
            ),
            'action': {move_to_point_cfg.name: [target]},
            'target': target,
        }
        run(**case)
    except Exception as e:
        print(f'exception is {e}')
        import sys
        import traceback

        traceback.print_exc()
        sys.exit(1)
