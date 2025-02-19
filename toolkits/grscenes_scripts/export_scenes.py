"""This script is used to export specific one or more scenes from the source full scenes directory.
Usage:
    python export_scenes.py -i {source_full_scenes_parent_dir} -o {target_few_scenes_parent_dir} -n {specific_scene_names...}
    e.g. python export_scenes.py -i /ssd/tianshihan/target_69_new -o /ssd/tianshihan/exported_3 -n MWAX5JYKTKJZ2AABAAAAACA8_usd MVUHLWYKTKJ5EAABAAAAADA8_usd MVUHLWYKTKJ5EAABAAAAADI8_usd
"""
import argparse
import json
import os
import re
import shutil

import tqdm
from pxr import Sdf, Usd


def find_texture_path_in_mdl(mdl_file_path):
    with open(mdl_file_path, 'r') as file:
        lines = file.readlines()
    # texture_2d("./Textures/white.png", texture_2d\("([^"]+)"
    pattern = re.compile(r'texture_2d\("([^"]+)",')

    result = set()

    for line in lines:
        match = re.findall(pattern, line)
        if match:
            result.update(match)

    return result


def get_model_reference(prim):
    model_refs = set()
    references = prim.GetMetadata('references')
    for ref in references.GetAddedOrExplicitItems():
        if isinstance(ref, Sdf.Reference):
            model_refs.add(str(ref.assetPath))

    return model_refs


def get_material_reference(stage, prim):
    material_refs = set()
    material_bindings = prim.GetRelationship('material:binding').GetTargets()
    for material_path in material_bindings:
        material_prim = stage.GetPrimAtPath(material_path)
        if material_prim:
            for shader in material_prim.GetChildren():
                for shader_property in shader.GetProperties():
                    if shader_property.GetTypeName() == 'asset':
                        material_refs.add(str(shader_property.Get().path))

                shader_mdl_absolute_path = shader.GetProperty('info:mdl:sourceAsset').Get().resolvedPath
                if shader_mdl_absolute_path:
                    texture_path_set = find_texture_path_in_mdl(shader_mdl_absolute_path)
                    material_refs.update(
                        [os.path.join('./Materials', texture_path) for texture_path in texture_path_set]
                    )

    return material_refs


def get_model_and_material_references_set(stage):
    model_refs_list = set()
    material_refs_list = set()
    for prim in stage.Traverse():
        if prim.HasMetadata('references'):
            model_ref = get_model_reference(prim)
            if model_ref:
                model_refs_list.update(model_ref)

        if prim.HasRelationship('material:binding'):
            material_ref = get_material_reference(stage, prim)
            if material_ref:
                material_refs_list.update(material_ref)

        if prim.GetName() == 'textures':
            for attr in prim.GetAttributes():
                if attr.GetTypeName() == 'asset':
                    material_refs_list.add(str(attr.Get().path))

    material_refs_list.add('Materials/OmniUe4Base.mdl')
    material_refs_list.add('Materials/OmniUe4Function.mdl')
    material_refs_list.add('Materials/OmniUe4Translucent.mdl')
    material_refs_list.add('Materials/WorldGridMaterial.mdl')
    material_refs_list.add('Materials/DayMaterial.mdl')
    material_refs_list.add('Materials/KooPbr_maps.mdl')
    material_refs_list.add('Materials/KooPbr.mdl')

    return model_refs_list, material_refs_list


def parse_into_json_file(refs_list, json_file_path):
    with open(json_file_path, 'w') as f:
        json.dump(refs_list, f, indent=4)


def copy_file(src, dst):
    try:
        if not os.path.exists(src):
            print(f'Error! File {src} not found!')
            return

        dst_dir = os.path.dirname(dst)
        if not os.path.exists(dst_dir):
            os.makedirs(dst_dir)

        shutil.copyfile(src, dst)
    except Exception:
        print(f'Error! Copy file from {src} to {dst} failed!')


def export(src_dir, target_dir, scene_name_list):
    for scene_name in tqdm.tqdm(scene_name_list, desc='Exporting scene'):
        scene_dir = os.path.join(src_dir, 'scenes', scene_name)
        target_scene_dir = os.path.join(target_dir, 'scenes', scene_name)

        usd_files = [
            f for f in os.listdir(scene_dir) if f.endswith('.usd') or f.endswith('.usda') or f.endswith('.usdz')
        ]
        stage_initial_usd = os.path.join(scene_dir, 'start_result_raw.usd')
        # get all references of this stage
        stage = Usd.Stage.Open(stage_initial_usd)
        model_refs_list, material_refs_list = get_model_and_material_references_set(stage)
        # parse_into_json_file(list(model_refs_list), "/ssd/tianshihan/test/model_refs_test.json")
        # parse_into_json_file(list(material_refs_list), "/ssd/tianshihan/test/material_refs_test.json")
        # export(copy) all references from source to target path
        for model_ref in model_refs_list:
            copy_file(os.path.join(src_dir, model_ref), os.path.join(target_dir, model_ref))
        for material_ref in material_refs_list:
            copy_file(os.path.join(src_dir, material_ref), os.path.join(target_dir, material_ref))
        # export(copy) usd files from source to target path
        for usd_file in usd_files:
            copy_file(os.path.join(scene_dir, usd_file), os.path.join(target_scene_dir, usd_file))

    # create "Materials" symlink with model instance.usd
    for root, dirs, files in os.walk(os.path.join(target_dir, 'models')):
        for _ in files:
            link_name = os.path.join(root, 'Materials')
            target_material_dir = os.path.join(target_dir, 'Materials')
            target_material_dir_relpath = os.path.relpath(target_material_dir, root)
            os.symlink(target_material_dir_relpath, link_name)

    # create "Materials" and "models" symlink with stage start_result_interaction.usd
    for root, dirs, files in os.walk(os.path.join(target_dir, 'scenes')):
        for _ in files:
            materials_link_name = os.path.join(root, 'Materials')
            models_link_name = os.path.join(root, 'models')
            target_material_dir = os.path.join(target_dir, 'Materials')
            target_material_dir_relpath = os.path.relpath(target_material_dir, root)
            target_model_dir = os.path.join(target_dir, 'models')
            target_model_dir_relpath = os.path.relpath(target_model_dir, root)
            if not os.path.islink(materials_link_name):
                os.symlink(target_material_dir_relpath, materials_link_name)
            if not os.path.islink(models_link_name):
                os.symlink(target_model_dir_relpath, models_link_name)


parser = argparse.ArgumentParser(description='Export some scenes from the source directory to the target directory.')
parser.add_argument(
    '-i', '--input', required=True, help='the source parent directory path of scenes, e.g. /ssd/tianshihan/target'
)
parser.add_argument(
    '-o', '--output', required=True, help='the target parent directory path of scenes, e.g. /ssd/tianshihan/target_new'
)
parser.add_argument('-n', '--names', required=True, nargs='*', help='the scene names to export')

args = parser.parse_args()
src_dir = os.path.abspath(args.input)
print('Source scenes parent dir: ', src_dir)
target_dir = os.path.abspath(args.output)
print('Target scenes parent dir: ', target_dir)
scene_name_list = args.names

export(src_dir, target_dir, scene_name_list)
