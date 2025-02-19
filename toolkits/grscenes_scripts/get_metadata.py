"""This script is used to get metadata (material & model reference paths) for model or scene assets.
The metadata results will be saved as a file named `metadata.json` under the parent directory of the specific model or scene usd assets.
Usage:
    python get_metadata.py -f {specific_model_usd... or specific_scene_usd...}
    e.g. python get_metadata.py -f /ssd/tianshihan/target_69_new/models/object/others/bed/fed722160376b16aee586d0790876b56/instance.usd
    e.g. python get_metadata.py -f /ssd/tianshihan/target_69_new/scenes/MV7J6NIKTKJZ2AABAAAAADA8_usd/start_result_raw.usd

    python get_metadata.py -d {specific_model_folder... or specific_scene_folder...}
    e.g. python get_metadata.py -d /ssd/tianshihan/target_69_new/models
    e.g. python get_metadata.py -d /ssd/tianshihan/target_69_new/scenes
"""
import argparse
import json
import os
import re

from pxr import Sdf, Usd


def find_texture_path_in_mdl(mdl_file_path):
    with open(mdl_file_path, 'r') as file:
        lines = file.readlines()
    # texture_2d("./Textures/white.png", ...
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

    return model_refs_list, material_refs_list


def parse_metadata(usd_abs_path):
    stage = Usd.Stage.Open(usd_abs_path)
    model_refs_list, material_refs_list = get_model_and_material_references_set(stage)
    metadata_dict = {'models': list(model_refs_list), 'materials': list(material_refs_list)}
    metedata_json_path = os.path.join(os.path.dirname(usd_abs_path), 'metadata.json')
    with open(metedata_json_path, 'w') as f:
        json.dump(metadata_dict, f, indent=4)

    return metedata_json_path


parser = argparse.ArgumentParser(
    description='Get metadata (material & model reference paths) for model or scene assets.'
)
parser.add_argument('-f', '--files', required=False, nargs='*', help='the usd file(s) to get metadata')
parser.add_argument('-d', '--dirs', required=False, nargs='*', help='the folders including usd file(s) to get metadata')

args = parser.parse_args()

if not args.files and not args.dirs:
    print('No path specified!')
else:
    if args.files:
        for f in args.files:
            f_abs_path = os.path.abspath(f)
            if not os.path.exists(f_abs_path):
                print(f'Error! {f} not found!')
            else:
                metadata_json_path = parse_metadata(f_abs_path)
                print(f'The metadata of {f_abs_path} is saved into {metadata_json_path}')

    if args.dirs:
        for d in args.dirs:
            d_abs_path = os.path.abspath(d)
            for root, _, files in os.walk(d_abs_path):
                for f in files:
                    if f == 'instance.usd' or f == 'start_result_raw.usd':
                        f_abs_path = os.path.join(root, f)
                        metadata_json_path = parse_metadata(f_abs_path)
                        print(f'The metadata of {f_abs_path} is saved into {metadata_json_path}')
