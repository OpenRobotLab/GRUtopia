import os

from internutopia.download_assets import unzip_all
from internutopia.macros import gm

RED = '\033[31m'
GREEN = '\033[32m'
YELLOW = '\033[33m'
BLUE = '\033[34m'
END = '\033[0m'


def main():
    print(f'Current assets path: {gm.ASSET_PATH}')
    target_path = ''
    while True:
        target_path = input('Please enter the new assets path (must be absolute path): ').strip()
        if target_path.startswith('/'):
            break
        print('target path must be absolute path')
    if not os.path.isdir(target_path):
        print(
            f'{RED}ERROR{END}: {target_path} is not an existing directory.\nPlease ensure the assets have been placed at {target_path}.'
        )
        return

    config_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'default_config.py')
    with open(config_file, 'w') as f:
        f.write(f'DEFAULT_ASSETS_PATH = "{target_path}"')
    print(f'Assets path has been set to: {target_path}')

    unzip = input('Need to unzip all the assets? (assets should only be unzipped once) (y/N) ').strip().lower()
    if unzip == 'y':
        unzip_all(target_path)


if __name__ == '__main__':
    main()
