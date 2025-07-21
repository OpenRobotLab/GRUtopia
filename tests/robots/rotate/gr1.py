from test_rotate import run

from internutopia_extension.configs.robots.gr1 import GR1RobotCfg, rotate_cfg

if __name__ == '__main__':
    try:
        target = (3.0, 2.0, 0.0)
        case = {
            'robot': GR1RobotCfg(
                position=(0.0, 0.0, 0.95),
                controllers=[rotate_cfg],
            ),
            'action': {rotate_cfg.name: [(0, 0, 0, 1)]},
            'target': (0, 0, 0, 1),
            'threshold': rotate_cfg.threshold,
        }
        run(**case)
    except Exception as e:
        print(f'exception is {e}')
        import sys
        import traceback

        traceback.print_exc()
        sys.exit(1)
