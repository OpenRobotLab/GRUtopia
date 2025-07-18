from test_move_to_point import run

from grutopia_extension.configs.robots.h1_with_hand import (
    H1WithHandRobotCfg,
    move_to_point_cfg,
)

if __name__ == '__main__':
    try:
        target = (3.0, 2.0, 0.0)
        case = {
            'robot': H1WithHandRobotCfg(
                position=(0.0, 0.0, 1.05),
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
