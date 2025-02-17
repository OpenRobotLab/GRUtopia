# Installation

## Prerequisites

- OS: Ubuntu 20.04+
- RAM: 32GB+
- GPU: NVIDIA RTX 2070+
- NVIDIA Driver: 525.85+

> GRUtopia is built upon NVIDIA's [Omniverse](https://www.nvidia.com/en-us/omniverse/) and [Isaac Sim](https://developer.nvidia.com/isaac-sim) platforms, so we inherit their dependencies. For more information, please see [Isaac Sim's Requirements](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/requirements.html).

Two ways of installation are provided:

- Install from source (Linux): [workstation installation](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_workstation.html) of Isaac Sim is required, and is recommended for users who wants to run Isaac Sim as a GUI application on Linux workstation with a GPU.
- Install with Docker (Linux): [container installation](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_container.html) of Isaac Sim is required, and is recommended for deployment on remote headless servers or the Cloud using a Docker container running Linux.

See more: [Differences Between Workstation And Docker](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_faq.html#isaac-sim-setup-differences).

Windows support is in our roadmap. Contributions are welcome!

## Install from source (Linux)

Make sure you have [Isaac Sim 4.1.0](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_workstation.html) installed. Please use version 4.1.0 instead of any other versions, such as 4.0.0 or 4.2.0, to avoid potential issues and ensure optimal performance.

[Conda](https://conda.io/projects/conda/en/latest/user-guide/install/index.html) is required to install from source.

1. Navigate to Isaac Sim root path (default path in Ubuntu is `$HOME/.local/share/ov/pkg/isaac_sim-4.1.0`) and clone the repository.

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

   If properly installed, Isaac Sim GUI window should pop up and you can see a humanoid robot (Unitree H1) walking following a pre-defined trajectory in Isaac Sim.

## Install with Docker (Linux)

Make sure you have [Docker](https://docs.docker.com/get-docker/) and [NVIDIA Container Toolkit](https://github.com/NVIDIA/nvidia-container-toolkit) installed. You can refer to the [container installation doc](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_container.html) of Isaac Sim for detailed instructions.

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
   $ docker pull nvcr.io/nvidia/isaac-sim:4.1.0
   ```
2. Build docker image, replacing <your tag> with your desired tag:

   ```bash
   $ cd PATH/TO/GRUTOPIA/ROOT

   $ docker build -t grutopia:<your tag> .
   ```

3. Start docker container, replacing <your tag> with the above tag:

   ```bash
   $ cd PATH/TO/GRUTOPIA/ROOT

   $ export CACHE_ROOT=$HOME/docker  # set cache root path
   $ export WEBUI_HOST=127.0.0.1  # set webui listen address, default to 127.0.0.1

   $ docker run --name grutopia -it --rm --gpus all --network host \
     -e "ACCEPT_EULA=Y" \
     -e "PRIVACY_CONSENT=Y" \
     -e "WEBUI_HOST=${WEBUI_HOST}" \
     -v ${PWD}:/isaac-sim/GRUtopia \
     -v ${CACHE_ROOT}/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
     -v ${CACHE_ROOT}/isaac-sim/cache/ov:/root/.cache/ov:rw \
     -v ${CACHE_ROOT}/isaac-sim/cache/pip:/root/.cache/pip:rw \
     -v ${CACHE_ROOT}/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
     -v ${CACHE_ROOT}/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
     -v ${CACHE_ROOT}/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
     -v ${CACHE_ROOT}/isaac-sim/data:/root/.local/share/ov/data:rw \
     -v ${CACHE_ROOT}/isaac-sim/documents:/root/Documents:rw \
     grutopia:<your tag>
   ```

4. Verify the Installation.

   Run inside container:

   ```bash
   # run inside container
   $ python ./GRUtopia/demo/h1_locomotion.py  # start simulation
   ```

   If properly installed, observation from simulation will be displayed in the terminal every 100 steps, and you can access the Isaac Sim through WebRTC at <http://127.0.0.1:8211/streaming/webrtc-demo/?server=127.0.0.1> (if you have set a different `WEBUI_HOST`, use that instead of `127.0.0.1`).
