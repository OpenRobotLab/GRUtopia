import os
import random
import shutil
import string
import subprocess
import zipfile

from grutopia.core.util import is_in_container
from grutopia.macros import gm

# Default environment name
DEFAULT_ENV_NAME = ''

minimum_exclude_path_patterns = [
    'scenes/GRScenes-100/*',
    'benchmark/*',
]

minimum_include_paths = [
    '/objects/',
    '/robots/',
    '/scenes/empty.usd',
    '/scenes/demo_scenes',
    '/scene.png',
    '/README.md',
]


def generate_random_env_name() -> str:
    prefix = 'grutopia_asset_'
    random_string = ''.join(random.choices(string.ascii_letters + string.digits, k=8))
    return f'{prefix}_{random_string}'


def create_conda_env():
    try:
        print(f"Creating Conda environment '{DEFAULT_ENV_NAME}' for downloading dataset...")
        subprocess.check_call(['conda', 'create', '--name', DEFAULT_ENV_NAME, '--yes', 'python=3.10'])
    except subprocess.CalledProcessError as e:
        print(f"Error creating Conda environment '{DEFAULT_ENV_NAME}': {e}")
        raise


def download_via_openxlab(target_path, minimum=False):
    args_prefix = []
    # Install package
    if DEFAULT_ENV_NAME != '':
        print('Installing openxlab via pip...')
        subprocess.check_call(['conda', 'run', '-n', DEFAULT_ENV_NAME, 'pip', 'install', 'openxlab==0.1.2'])
        args_prefix = [
            'conda',
            'run',
            '--no-capture-output',
            '-n',
            DEFAULT_ENV_NAME,
        ]

    # Check login
    auth_path = os.path.expanduser('~/.openxlab/')
    if not os.path.exists(auth_path):
        print(
            'Downloading from openxlab requires AK/SK, which can be found at https://sso.openxlab.org.cn/usercenter?tab=secret after login.'
        )
        ak = input('Enter your openxlab Access Key (AK): ')
        sk = input('Enter your openxlab Secret Key (SK): ')

        # Set the environment variables
        os.environ['OPENXLAB_AK'] = ak
        os.environ['OPENXLAB_SK'] = sk
        subprocess.check_call(
            args_prefix
            + [
                'python',
                os.path.join(os.path.dirname(os.path.abspath(__file__)), 'login_openxlab.py'),
            ]
        )

    # openxlab dataset get --dataset-repo OpenRobotLab/GRScenes  --target-path <target_path>
    print('Downloading assets...')
    dataset_repo = 'OpenRobotLab/GRScenes'
    if not minimum:
        subprocess.check_call(
            args_prefix
            + [
                'openxlab',
                'dataset',
                'get',
                '--dataset-repo',
                dataset_repo,
                '--target-path',
                target_path,
            ]
        )
        return
    # Download minimum dataset.
    base_args = args_prefix + [
        'openxlab',
        'dataset',
        'download',
        '--dataset-repo',
        dataset_repo,
        '--target-path',
        target_path,
    ]
    for path in minimum_include_paths:
        args = base_args + ['--source-path', path]
        print(f'Downloading {path}...')
        subprocess.check_call(args)

    # There will be an unnecessary directory created after download, remove it
    print('Removing unnecessary directory...')
    asset_dir = os.path.join(target_path, dataset_repo.replace('/', '___'))
    for file_name in os.listdir(asset_dir):
        shutil.move(os.path.join(asset_dir, file_name), target_path)
    remove_dir(asset_dir)


def download_via_huggingface(target_path, minimum=False):
    args_prefix = []
    # Install package
    if DEFAULT_ENV_NAME != '':
        print('Installing huggingface-hub via pip...')
        subprocess.check_call(['conda', 'run', '-n', DEFAULT_ENV_NAME, 'pip', 'install', 'huggingface-hub==0.28.1'])
        args_prefix = [
            'conda',
            'run',
            '--no-capture-output',
            '-n',
            DEFAULT_ENV_NAME,
        ]

    # huggingface-cli download OpenRobotLab/GRScenes --repo-type dataset --local-dir <target_path>
    print('Downloading assets...')
    dataset_repo = 'OpenRobotLab/GRScenes'
    args = args_prefix + [
        'huggingface-cli',
        'download',
        dataset_repo,
        '--repo-type',
        'dataset',
        '--local-dir',
        target_path,
    ]
    if minimum:
        args.extend(['--exclude'] + minimum_exclude_path_patterns)
    subprocess.check_call(args)


def download_via_modelscope(target_path, minimum=False):
    args_prefix = []
    # Install package
    if DEFAULT_ENV_NAME != '':
        print('Installing modelscope via pip...')
        subprocess.check_call(['conda', 'run', '-n', DEFAULT_ENV_NAME, 'pip', 'install', 'modelscope==1.23.0'])
        args_prefix = [
            'conda',
            'run',
            '--no-capture-output',
            '-n',
            DEFAULT_ENV_NAME,
        ]

    # modelscope download --dataset Shanghai_AI_Laboratory/GRScenes --local_dir <target_path>
    print('Downloading assets...')
    dataset_repo = 'Shanghai_AI_Laboratory/GRScenes'
    args = args_prefix + [
        'modelscope',
        'download',
        '--dataset',
        dataset_repo,
        '--local_dir',
        target_path,
    ]
    if minimum:
        args.extend(['--exclude'] + minimum_exclude_path_patterns)
    subprocess.check_call(args)


def remove_dir(dir_path):
    if os.path.exists(dir_path):
        shutil.rmtree(dir_path)


def delete_conda_env():
    try:
        print(f"Deleting Conda environment '{DEFAULT_ENV_NAME}'...")
        subprocess.check_call(['conda', 'env', 'remove', '--name', DEFAULT_ENV_NAME, '--yes'])
    except subprocess.CalledProcessError as e:
        print(f"Error deleting Conda environment '{DEFAULT_ENV_NAME}': {e}")


def unzip_all(dir_path):
    if not os.path.exists(dir_path):
        return

    print('Unzipping...')
    for root, _, files in os.walk(dir_path):
        for file in files:
            if file.endswith('.zip'):
                zip_path = os.path.join(root, file)
                with zipfile.ZipFile(zip_path, 'r') as zip_ref:
                    zip_ref.extractall(root)
                print(f'Extracted: {zip_path} to {root}')

    # Rename directory target_69_new to home_scenes, target_30_new to commercial_scenes
    print('Renaming...')
    for root, dirs, files in os.walk(dir_path):
        for dir in dirs:
            if dir == 'target_69_new':
                rename(root, 'target_69_new', 'home_scenes')
            if dir == 'target_30_new':
                rename(root, 'target_30_new', 'commercial_scenes')


def rename(root, old_name, new_name) -> str:
    old_path = os.path.join(root, old_name)
    new_path = os.path.join(root, new_name)
    os.rename(old_path, new_path)


def cleanup_process(dataset_src: str):
    print('Cleanup downloading process')
    subprocess.check_call(
        f'ps -ef | grep {DEFAULT_ENV_NAME} | grep {dataset_src} |' + " awk '{print $2}' | xargs kill",
        shell=True,
    )


def main():
    print('Welcome to the Dataset Downloader!')

    # Validate dataset source
    dataset_src = (
        input('Please choose the dataset source for download (huggingface/modelscope/openxlab): ').strip().lower()
        or 'huggingface'
    )
    if dataset_src not in ['huggingface', 'modelscope', 'openxlab']:
        print("Invalid dataset source. Please choose 'huggingface' or 'modelscope' or 'openxlab'.")
        return

    # Determine the target path
    target_path = gm.ASSET_PATH
    full = (
        input(
            """
Options:
    - min (~500MiB): only a minimum set of assets will be downloaded, without the GRScenes-100 and assets required by benchmark baselines.
    - full: (~80GiB to download, ~230GiB after unzipped): full assets will be downloaded.
Please choose (min/full): """
        )
        .strip()
        .lower()
    )
    minimum = True
    if full != 'min':
        full = 'full'
        minimum = False
    print(f'Asset ({full}) will be installed under this location: {target_path}')
    desired_path = input('If you want to use a different one, please type in (must be absolute path): ').strip()
    if desired_path != '':
        target_path = desired_path
        while True:
            if target_path.startswith('/'):
                break
            print('desired path must be absolute path')
            target_path = input('Please enter the desired path : ').strip()
        # Persistent the target path.
        config_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'default_config.py')
        with open(config_file, 'w') as f:
            f.write(f'DEFAULT_ASSETS_PATH = "{target_path}"')
        print(f'Assets will be installed under this location: {target_path}')
    path_confirm = input('Do you want to continue? (Y/n): ').strip().lower()
    if path_confirm not in ['y', 'yes']:
        return

    # Check if dataset already exists
    if os.path.exists(target_path):
        print(f'Dataset already exists at {target_path}. No need to download again.')
        return

    # Create a conda env for download
    if not is_in_container():
        global DEFAULT_ENV_NAME
        DEFAULT_ENV_NAME = generate_random_env_name()
        create_conda_env()

    try:
        print('Starting to download assets...')
        if dataset_src == 'huggingface':
            download_via_huggingface(target_path, minimum)
        if dataset_src == 'modelscope':
            download_via_modelscope(target_path, minimum)
        if dataset_src == 'openxlab':
            download_via_openxlab(target_path, minimum)

        # Unzip the dataset
        unzip_all(target_path)

    except Exception as e:
        print(f'Error downloading assets: {e}')
        print('Removing downloaded assets files...')
        remove_dir(target_path)
    except KeyboardInterrupt:
        if DEFAULT_ENV_NAME != '':
            cleanup_process(dataset_src)
    finally:
        # Remove the conda env
        if DEFAULT_ENV_NAME != '':
            delete_conda_env()


if __name__ == '__main__':
    main()
