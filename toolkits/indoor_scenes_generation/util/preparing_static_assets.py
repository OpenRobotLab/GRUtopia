from isaacsim import SimulationApp

simulation_app = SimulationApp({'headless': True})

import argparse
import os

from pxr import Gf, Usd


def process_usd_file(usd_file_path, output_folder, index_dict):
    """
    Processes a given USD file by scaling and rotating its contents,
    then saves it to a new location.

    Parameters:
    - usd_file_path (str): The path to the input USD file to process.
    - output_folder (str): The folder where the processed file will be saved.
    - index_dict (dict): A dictionary to track object occurrences to prevent duplication.

    Returns:
    None
    """
    # Extract object name from file path
    obj_name = usd_file_path.split('/')[-3].replace('_', '').title()
    index_dict[obj_name] = index_dict.get(obj_name, 0) + 1

    # Open the USD file
    stage = Usd.Stage.Open(usd_file_path)

    # Apply scaling & rotation transformation
    xform_prim = stage.GetPrimAtPath('/Root/Instance')

    scale = xform_prim.GetAttribute('xformOp:scale')
    scale.Set(Gf.Vec3d(0.001, 0.001, 0.001))

    orient = xform_prim.GetAttribute('xformOp:orient')
    orient.Set((Gf.Rotation(Gf.Vec3d(1, 0, 0), -90) * Gf.Rotation(Gf.Vec3d(0, 1, 0), 90)).GetQuat())

    # Export the processed USD file to the new path
    file_name = f'{obj_name}_{index_dict[obj_name]}.usd'
    output_file_path = os.path.join(output_folder, obj_name, file_name)

    directory = os.path.dirname(output_file_path)
    if not os.path.exists(directory):
        os.makedirs(directory)

    stage.Export(output_file_path)


def traverse_folders(input_folder, output_folder):
    """
    Traverses the input folder to find all USD files and processes them.

    Parameters:
    - input_folder (str): The folder containing the input USD files.
    - output_folder (str): The folder where the processed files will be saved.

    Returns:
    None
    """
    index_dict = {}

    for root, dirs, files in os.walk(input_folder):
        for file in files:
            if file.endswith('.usd'):
                usd_file_path = os.path.join(root, file)
                process_usd_file(usd_file_path, output_folder, index_dict)


def main():
    """
    Main function to parse arguments and start the static assets preparation process.
    """
    parser = argparse.ArgumentParser(description='Prepare static assets for Infinigen from GRScenes100 dataset.')
    parser.add_argument('--input_folder', type=str, help='Path to the GRScenes100 dataset folder with object models.')
    parser.add_argument('--output_folder', type=str, help='Path to the destination folder for processed assets.')

    args = parser.parse_args()

    # Traverse the input folder and process USD files
    traverse_folders(args.input_folder, args.output_folder)


if __name__ == '__main__':
    main()
