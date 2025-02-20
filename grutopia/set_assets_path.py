import os


def main():
    target_path = ''
    while True:
        target_path = input('Please enter the target path (must be absolute path): ').strip()
        if target_path.startswith('/'):
            break
        print('target path must be absolute path')
    try:
        os.stat(target_path)
    except Exception as e:
        print(f'Cannot stat path {target_path}: {e}\nPlease ensure the assets have been placed at {target_path}.')
    else:
        config_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'default_config.py')
        with open(config_file, 'w') as f:
            f.write(f'DEFAULT_ASSETS_PATH = "{target_path}"')
        print(f'Assets path has been set to: {target_path}')


if __name__ == '__main__':
    main()
