# Installation

## Prerequisites

- OS: Ubuntu 20.04+
- RAM: 32GB+
- GPU: NVIDIA RTX 2070+
- NVIDIA Driver: 525.85+

## Install from source (Linux)

Make sure you have [Isaac Sim 2023.1.1](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_workstation.html) installed.

[Conda](https://conda.io/projects/conda/en/latest/user-guide/install/index.html) is required to install from source.

1. Navigate to Isaac Sim root path (default path in Ubuntu is `$HOME/.local/share/ov/pkg/isaac_sim-2023.1.1`) and clone the repository.

   ```bash
   $ cd PATH/TO/ISAAC_SIM/ROOT
   $ git clone git@github.com:OpenRobotLab/GRUtopia.git
   ```

1. Download [dataset](https://openxlab.org.cn/datasets/OpenRobotLab/GRScenes/cli/main) and save it to the `assets` directory under GRUtopia root path.

   The file structure should be like:

   ```
   GRUtopia
   ├── assets
   │   ├── objects
   │   ├── policy
   │   ├── robots
   │   └── scenes
   ├── demo
   │   ├── configs
   │   ├── h1_city.py
   │   ├── h1_locomotion.py
   │   └── h1_npc.py
   ...
   ```

1. Navigate to GRUtopia root path and configure the conda environment.

   ```bash
   $ cd PATH/TO/GRUTOPIA/ROOT

   # Conda environment will be created and configured automatically with prompt.
   $ ./setup_conda.sh

   $ cd .. && conda activate grutopia  # or your conda env name
   ```

1. Verify the Installation.

   Run at the root path of Isaac Sim:

   ```bash
   $ cd PATH/TO/ISAAC_SIM/ROOT
   $ python ./GRUtopia/demo/h1_locomotion.py  # start simulation
   ```

## Install with Docker (Linux)

Make sure you have [Docker](https://docs.docker.com/get-docker/) installed.

1. Clone the GRUtopia repository to any desired location.

   ```bash
   $ git clone git@github.com:OpenRobotLab/GRUtopia.git
   ```

1. Download [dataset](https://openxlab.org.cn/datasets/OpenRobotLab/GRScenes/cli/main) and save it to the `assets` directory under GRUtopia root path.

   The file structure should be like:

   ```
   GRUtopia
   ├── assets
   │   ├── objects
   │   ├── policy
   │   ├── robots
   │   └── scenes
   ├── demo
   │   ├── configs
   │   ├── h1_city.py
   │   ├── h1_locomotion.py
   │   └── h1_npc.py
   ...
   ```

1. Pull the Isaac Sim image (`docker login` is required, please refer to [NGC Documents](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/isaac-sim)).

   ```bash
   $ docker pull nvcr.io/nvidia/isaac-sim:2023.1.1
   ```
1. Build docker image.

   ```bash
   $ cd PATH/TO/GRUTOPIA/ROOT

   $ docker build -t grutopia:0.0.1 .
   ```

1. Start docker container.

   ```bash
   $ cd PATH/TO/GRUTOPIA/ROOT

   $ export CACHE_ROOT=$HOME/docker  # set cache root path
   $ export WEBUI_HOST=127.0.0.1  # set webui listen address, default to 127.0.0.1

   $ docker run --name grutopia -it --rm --gpus all --network host \
     -e "ACCEPT_EULA=Y" \
     -e "PRIVACY_CONSENT=Y" \
     -e "WEBUI_HOST=${WEBUI_HOST}" \
     -v ${PWD}:/isaac-sim/GRUtopia \
     -v ${PWD}/test/.test_scripts:/isaac-sim/run_scripts \
     -v ${CACHE_ROOT}/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
     -v ${CACHE_ROOT}/isaac-sim/cache/ov:/root/.cache/ov:rw \
     -v ${CACHE_ROOT}/isaac-sim/cache/pip:/root/.cache/pip:rw \
     -v ${CACHE_ROOT}/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
     -v ${CACHE_ROOT}/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
     -v ${CACHE_ROOT}/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
     -v ${CACHE_ROOT}/isaac-sim/data:/root/.local/share/ov/data:rw \
     -v ${CACHE_ROOT}/isaac-sim/documents:/root/Documents:rw \
     grutopia:0.0.1
   ```

1. Verify the Installation.

   Run inside container:

   ```bash
   # run inside container
   $ python ./GRUtopia/demo/h1_locomotion.py  # start simulation
   ```
