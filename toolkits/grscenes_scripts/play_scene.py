"""This script is used to load and play a scene in the Isaac Sim GUI.

Usage:
    python play_scene.py -f {scene_usd_file}
"""
import argparse
import os

parser = argparse.ArgumentParser(description='Load and play the scene in the Isaac Sim GUI.')
parser.add_argument('-f', '--file', required=True, type=str, help='the usd file to play')

args = parser.parse_args()

f_abs_path = os.path.abspath(args.file)
if not os.path.exists(f_abs_path):
    print(f'Error! {args.file} not found!')
else:
    from isaacsim import SimulationApp

    CONFIG = {'headless': False}
    kit = SimulationApp(launch_config=CONFIG)

    import omni.physx.bindings._physx as physx_bindings

    kit.set_setting(physx_bindings.SETTING_UJITSO_COLLISION_COOKING, False)

    kit.context.open_stage(f_abs_path)
    stage = kit.context.get_stage()
    kit._wait_for_viewport()

    import omni.timeline

    omni.timeline.get_timeline_interface().play()
    while kit.is_running():
        kit.update()
