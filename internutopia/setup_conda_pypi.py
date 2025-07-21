import os


def main():
    # Check if the ISAAC_SIM_PATH variable is set correctly
    pkg_path = os.path.expanduser('~/.local/share/ov/pkg')

    if os.path.isdir(pkg_path) and any('isaac' in d for d in os.listdir(pkg_path)):
        found_paths = [d for d in os.listdir(pkg_path) if 'isaac' in d]
        found_isaac_sim_path = os.path.join(pkg_path, found_paths[-1])
        print(
            f'We found Isaac Sim installed at \033[4m{found_isaac_sim_path}\033[0m. InternUtopia will use it by default.'
        )

        isaac_sim_path = input(
            'If you want to use a different one, please type in the path containing isaac-sim.sh here (press enter to skip) >>> '
        )
        isaac_sim_path = isaac_sim_path or found_isaac_sim_path
    else:
        print('We did not find Isaac Sim under ~/.local/share/ov/pkg.')
        print("If you haven't installed Isaac Sim yet, please do so before running this setup script.")
        isaac_sim_path = input(
            'If you have already installed it in a custom location, please type in the path containing isaac-sim.sh here >>> '
        )

    # Check if isaac-sim.sh exists
    while not os.path.isfile(os.path.join(isaac_sim_path, 'isaac-sim.sh')):
        isaac_sim_path = input(
            f'isaac-sim.sh not found in \033[4m{isaac_sim_path}\033[0m! Make sure you have entered the correct path >>> '
        )

    print(f'\nUsing Isaac Sim at \033[4m{isaac_sim_path}\033[0m\n')

    # Create necessary directories and files
    conda_prefix = os.environ.get('CONDA_PREFIX')
    os.makedirs(os.path.join(conda_prefix, 'etc/conda/activate.d'), exist_ok=True)
    os.makedirs(os.path.join(conda_prefix, 'etc/conda/deactivate.d'), exist_ok=True)

    # Create activation script
    conda_act_file = os.path.join(conda_prefix, 'etc/conda/activate.d/env_vars.sh')
    with open(conda_act_file, 'w') as f:
        f.write('#!/bin/sh\n')
        f.write('export LD_LIBRARY_PATH_OLD=$LD_LIBRARY_PATH\n')
        f.write('export PYTHONPATH_OLD=$PYTHONPATH\n')
        f.write(f'source {isaac_sim_path}/setup_conda_env.sh\n')

    # Create deactivation script
    conda_deact_file = os.path.join(conda_prefix, 'etc/conda/deactivate.d/env_vars.sh')
    with open(conda_deact_file, 'w') as f:
        f.write('#!/bin/sh\n')
        f.write('export LD_LIBRARY_PATH=$LD_LIBRARY_PATH_OLD\n')
        f.write('export PYTHONPATH=$PYTHONPATH_OLD\n')
        f.write('unset ISAAC_PATH\n')
        f.write('unset CARB_APP_PATH\n')
        f.write('unset LD_LIBRARY_PATH_OLD\n')
        f.write('unset PYTHONPATH_OLD\n')

    print(
        f'\nSetup succeed! Please run \033[4mconda deactivate && conda activate {os.path.basename(conda_prefix)}\033[0m to activate the environment.\n'
    )


if __name__ == '__main__':
    main()
