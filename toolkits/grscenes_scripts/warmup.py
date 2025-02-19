"""This script is used to warm-up the simulation.
NOTE:
    If the local disk space or the preset local mesh cache size is not enough,
    the warm-up process is likely to fail.
"""
import argparse
import os

import tqdm

parser = argparse.ArgumentParser(description='Warmup the simulation.')
parser.add_argument(
    '-r',
    '--reset',
    required=False,
    action='store_true',
    help='If specified, it will release the old local mesh cache first.',
)
parser.add_argument('-f', '--files', required=False, nargs='*', help='the usd file(s) to warmup')
parser.add_argument('-d', '--dirs', required=False, nargs='*', help='the folders including usd file(s) to warmup')

args = parser.parse_args()

# Check cli args
if not args.files and not args.dirs:
    print('No path specified!')
    exit(1)

# SimulationApp should be initialized first before importing omni plugins
from isaacsim import SimulationApp

CONFIG = {'headless': True}
kit = SimulationApp(launch_config=CONFIG)

import omni.physx
import omni.physx.bindings._physx as physx_bindings
from omni.isaac.core import SimulationContext
from pxr import UsdUtils

kit.set_setting(physx_bindings.SETTING_UJITSO_COLLISION_COOKING, False)
kit.set_setting(physx_bindings.SETTING_USE_LOCAL_MESH_CACHE, True)
kit.set_setting(physx_bindings.SETTING_LOCAL_MESH_CACHE_SIZE_MB, 8192)

ujitso_cooking_enabled = kit._carb_settings.get_as_bool(physx_bindings.SETTING_UJITSO_COLLISION_COOKING)
use_local_cache = kit._carb_settings.get_as_bool(physx_bindings.SETTING_USE_LOCAL_MESH_CACHE)
local_cache_MB = kit._carb_settings.get(physx_bindings.SETTING_LOCAL_MESH_CACHE_SIZE_MB)

print(f'========== ujitso_cooking_enabled: {ujitso_cooking_enabled} ==========')
print(f'========== use_local_cache: {use_local_cache} ==========')
print(f'========== local_cache_MB: {local_cache_MB} ==========')


def warmup_simulation(stage, physics_scene_prim_path):
    stageId = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
    sim_context = SimulationContext(physics_prim_path=physics_scene_prim_path)
    sim_context._physics_context._physx_sim_interface.attach_stage(stageId)
    sim_context.play()
    sim_context.stop()


def test_single(scene_abs_path):
    kit.context.open_stage(scene_abs_path)
    stage = kit.context.get_stage()
    print(f'Warmup {scene_abs_path}')
    warmup_simulation(stage, '/Root/physicsScene')


def test_all(scene_abs_folder):
    scene_dirs = [_ for _ in os.listdir(scene_abs_folder) if os.path.isdir(os.path.join(scene_abs_folder, _))]
    for scene_dir in tqdm.tqdm(scene_dirs):
        navigation_path = os.path.join(scene_abs_folder, scene_dir, 'start_result_navigation.usd')
        interaction_path = os.path.join(scene_abs_folder, scene_dir, 'start_result_interaction.usd')
        test_single(navigation_path)
        test_single(interaction_path)


if __name__ == '__main__':
    # Release the old local mesh cache if specified
    if args.reset:
        omni.physx.get_physx_cooking_interface().release_local_mesh_cache()

    if args.files:
        for f in args.files:
            f_abs_path = os.path.abspath(f)
            if not os.path.exists(f_abs_path):
                print(f'Error! {f} not found!')
            else:
                test_single(f_abs_path)

    if args.dirs:
        for d in args.dirs:
            d_abs_path = os.path.abspath(d)
            if not os.path.exists(d_abs_path):
                print(f'Error! {d} not found!')
            else:
                test_all(d_abs_path)
