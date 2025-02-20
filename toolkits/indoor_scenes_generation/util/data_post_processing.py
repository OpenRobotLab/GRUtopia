from isaacsim import SimulationApp

simulation_app = SimulationApp({'headless': True})

import argparse
import os
import re

from pxr import Gf, Usd, UsdGeom


def check_string_and_file(input_string):
    """
    Check if the input string matches a specific pattern and extract object name and number.

    Parameters:
    - input_string (str): The string to check and process.

    Returns:
    - object_name (str): Extracted object name if valid, otherwise None.
    - number1 (str): Extracted number if valid, otherwise None.
    """
    # Define regex pattern for matching object name and number
    pattern = r'^([a-zA-Z]+)_(\d+)_(spawn_asset)_(\d+)_$'
    match = re.match(pattern, input_string)

    if match:
        # Extract object name and number
        object_name = match.group(1)
        index = match.group(2)
        return object_name, index
    else:
        return None, None


def process_usd_file(input_usd_path, source_usd_folder):
    """
    Process the USD file, applying transformations and referencing source assets.

    Parameters:
    - input_usd_path (str): Path to the input USD file to process.
    - source_usd_folder (str): Path to the folder containing the source assets.

    Returns:
    None
    """
    # Open the input USD file
    stage = Usd.Stage.Open(input_usd_path)
    if not stage:
        print(f'Cannot open USD file: {input_usd_path}')
        return

    # Get all Prim children under the root "/World"
    object_prims = stage.GetPrimAtPath('/World').GetChildren()

    for prim in object_prims:
        if prim.GetTypeName() == 'Xform':
            xform_name = prim.GetName()
            xform_path = prim.GetPath()

            object_name, index = check_string_and_file(xform_name)
            if not object_name:
                continue

            # Get the transformation for the Xform prim
            xform = UsdGeom.Xform(prim)
            time = Usd.TimeCode.Default()
            transformation = xform.GetLocalTransformation(time)

            # Construct the path to the target USD file
            target_usd_path = os.path.join(source_usd_folder, f'{object_name}/{object_name}_{index}.usd')
            if not os.path.exists(target_usd_path):
                print(f'Target file does not exist: {target_usd_path}')
                continue

            # Remove the original Xform
            stage.RemovePrim(xform_path)

            # Create a new Prim and reference the target USD file
            new_prim = stage.OverridePrim(xform_path)
            references = new_prim.GetReferences()
            references.AddReference(target_usd_path)

            new_xform = UsdGeom.Xform(new_prim)
            translation = transformation.ExtractTranslation()
            rotation = transformation.ExtractRotation()

            translate_op = new_xform.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
            translate_op.Set(translation)

            instance_xform_geom = new_xform.GetPrim().GetChild('Instance')
            instance_xform_geom.GetAttribute('xformOp:orient').Set(Gf.Quatd(1.0, Gf.Vec3d(0.0, 0.0, 0.0)))

            euler_rot = rotation.Decompose(
                Gf.Vec3d(1, 0, 0), Gf.Vec3d(0, 1, 0), Gf.Vec3d(0, 0, 1)  # X-axis  # Y-axis  # Z-axis
            )
            euler_rot[2] = euler_rot[2] + 90
            rotate_op = new_xform.AddRotateXYZOp(UsdGeom.XformOp.PrecisionDouble)
            rotate_op.Set(euler_rot)

            print(f'Processed Xform: {xform_path}, Applied transformations to: {target_usd_path}')

    # Save the modified stage to the target directory
    output_path = os.path.join(os.path.dirname(input_usd_path), 'final_scene.usd')
    stage.Export(output_path)
    print(f'Processing complete. Scene saved to: {output_path}')


def main():
    """
    Main function to parse arguments and run the USD post-processing.
    """
    parser = argparse.ArgumentParser(
        description='Post-process a USD scene by applying transformations and referencing source assets.'
    )
    parser.add_argument('--input_path', required=True, type=str, help='Path to the input USD file.')
    parser.add_argument(
        '--source_assets', required=True, type=str, help='Path to the folder containing the source assets.'
    )

    args = parser.parse_args()

    # Process the USD file
    process_usd_file(args.input_path, args.source_assets)


if __name__ == '__main__':
    main()
