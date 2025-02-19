import os

import openxlab


def login_to_openxlab(ak, sk):
    """Logs into OpenXLab with the provided Access Key and Secret Key."""
    try:
        openxlab.login(ak=ak, sk=sk)
        print('Login successful!')
    except Exception as e:
        print(f'Login failed: {e}')
        raise


if __name__ == '__main__':
    # Read AK and SK from environment variables
    ak = os.environ.get('OPENXLAB_AK')
    sk = os.environ.get('OPENXLAB_SK')

    if ak and sk:
        login_to_openxlab(ak, sk)
