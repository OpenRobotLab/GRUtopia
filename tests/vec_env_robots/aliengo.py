from test_move_to_point import run

from grutopia_extension.configs.robots.aliengo import AliengoRobotCfg, move_to_point_cfg

if __name__ == '__main__':
    try:
        target = (3.0, 2.0, 0.0)
        case = {
            'robot_0_cfg': (
                AliengoRobotCfg(
                    name='h1_0',
                    prim_path='/h1_0',
                    position=(0.0, 0.0, 1.05),
                    controllers=[move_to_point_cfg],
                ),
                {move_to_point_cfg.name: [(3.0, 2.0, 0.0)]},
                (3.0, 2.0, 0.0),
            ),
            'robot_1_cfg': (
                AliengoRobotCfg(
                    name='h1_1',
                    prim_path='/h1_1',
                    position=(3.0, 0.0, 1.05),
                    controllers=[move_to_point_cfg],
                ),
                {move_to_point_cfg.name: [(6.0, 2.0, 0.0)]},
                (6.0, 2.0, 0.0),
            ),
        }
        run(**case)
    except Exception as e:
        print(f'exception is {e}')
        import sys
        import traceback

        traceback.print_exc()
        sys.exit(1)
