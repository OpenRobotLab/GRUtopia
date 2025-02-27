# How to Use Controller

> This tutorial will show you how to use existing controllers for robots.

## What Is Controller

Controllers typically manage the joints of a robot and serve as the entries to robot actions. We utilize controllers to make a robot move and run.

## Supported Controllers

The directory `grutopia_extension/controllers/` contains a list of all supported controllers:

```
grutopia_extension/
└── controllers
    ├── aliengo_move_by_speed_controller.py
    ├── franka_mocap_teleop_controller.py
    ├── g1_move_by_speed_controller.py
    ├── gr1_move_by_speed_controller.py
    ├── gr1_teleop_controller.py
    ├── gripper_controller.py
    ├── h1_move_by_speed_controller.py
    ├── ik_controller.py
    ├── joint_controller.py
    ├── move_along_path_points_controller.py
    ├── move_to_point_by_speed_controller.py
    ├── move_to_point_oracle_controller.py
    ├── recover_controller.py
    ├── rmpflow_controller.py
    └── rotate_controller.py
```

For each robot, we provide some ready-to-use controller configurations in `grutopia_extension/configs/robots/{robot_name}.py`.

## How to Use a Controller

A controller must be used with a robot. So first of all, the controller configuration must be added to the controller list in robot configuration:

```
move_by_speed_cfg = MoveBySpeedCfg(...)

h1_1 = H1RobotCfg(
    position=(0.0, 0.0, 1.05),
    controllers=[
        move_by_speed_cfg,
    ],
）
```

Then in the main loop of the simulation, use the controller name as key, and corresponding action as value in the env action dict:

```Python
...
move_action = {move_by_speed_cfg.name: (1.0, 0.0, 0.0)}

while env.simulation_app.is_running():
    env_action = move_action
    obs, _, _, _, _ = env.step(actions=env_actions)
    ...
env.simulation_app.close()
```

It is important to specify the controllers in the robot before use. Please refer to the [how to use robot](./how-to-use-robot.md) for more detailed information.
