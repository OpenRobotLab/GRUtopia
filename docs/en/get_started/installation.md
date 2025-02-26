# Installation

## Prerequisites

- OS: Ubuntu 20.04+
- RAM: 32GB+
- GPU: NVIDIA RTX 2070+
- NVIDIA Driver: 525.85+

> GRUtopia is built upon NVIDIA's [Omniverse](https://www.nvidia.com/en-us/omniverse/) and [Isaac Sim](https://developer.nvidia.com/isaac-sim) platforms, inheriting their dependencies. GRUtopia 2.0.0 specifically requires Isaac Sim 4.2.0. To ensure optimal performance and avoid any potential issues, it is essential to use this version rather than any earlier releases, such as 4.1.0. For more information, please see [Isaac Sim's Requirements](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/requirements.html).

Two ways of installation are provided:

- Install from source (Linux): [workstation installation](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_workstation.html) of Isaac Sim is required, and is recommended for users who wants to run Isaac Sim as a GUI application on Linux workstation with a GPU.
- Install with Docker (Linux): [container installation](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_container.html) of Isaac Sim is required, and is recommended for deployment on remote headless servers or the Cloud using a Docker container running Linux.

See more: [Differences Between Workstation And Docker](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_faq.html#isaac-sim-setup-differences).

Windows support is in our roadmap. Contributions are welcome!


## Install from source (Linux)
There are two methods provided to install GRUtopia:
 - Installation from GitHub
 - Installation from PyPI

In addition, the [dataset](./how-to-use-grscenes.md) is provided alongside with a download script that supports dataset download via OpenXLab and HuggingFace. You can also [Prepare Dataset Manually](#prepare-dataset-manually).

Before proceeding with the installation, ensure that you have [Isaac Sim 4.2.0](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_workstation.html) and [Conda](https://conda.io/projects/conda/en/latest/user-guide/install/index.html) installed.

### Install GRUtopia from GitHub
1. Install GRUtopia
   ```bash
   $ git clone git@github.com:OpenRobotLab/GRUtopia.git
   ```

2. Navigate to GRUtopia root path and configure the conda environment.

   ```bash
   $ cd PATH/TO/GRUTOPIA/ROOT

   # Conda environment will be created and configured automatically with prompt.
   $ ./setup_conda.sh

   $ cd .. && conda activate grutopia  # or your conda env name
   ```
3. Download dataset to the specified directory.

   ```bash
   $ python grutopia/download_assets.py
   ```

2. Verify the Installation.

   ```bash
   $ python grutopia/demo/h1_locomotion.py  # start simulation
   ```

   If properly installed, Isaac Sim GUI window should pop up and you can see a humanoid robot (Unitree H1) walking following a pre-defined trajectory in Isaac Sim.

### Install GRUtopia from PyPI
1. Create conda env with `python=3.10` specified
    ```bash
   $ conda create -n <env> python=3.10
   ```
2. Install GRUtopia
   ```bash
   $ conda activate <env>
   $ pip install grutopia==2.0.0
   ```
3. Configure the conda environment.

   ```bash
   $ python -m grutopia.setup_conda_pypi

   $ conda deactivate && conda activate <env>
   ```
4. Download dataset to the specified directory.

   ```bash
   $  python -m grutopia.download_assets
   ```

5. Verify the Installation.

   ```bash
   $ python -m grutopia.demo.h1_locomotion  # start simulation
   ```

## Install with Docker (Linux)

Make sure you have [Docker](https://docs.docker.com/get-docker/) and [NVIDIA Container Toolkit](https://github.com/NVIDIA/nvidia-container-toolkit) installed. You can refer to the [container installation doc](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_container.html) of Isaac Sim for detailed instructions.

1. Clone the GRUtopia repository to any desired location.

   ```bash
   $ git clone git@github.com:OpenRobotLab/GRUtopia.git
   ```

1. Pull the Isaac Sim image (`docker login` is required, please refer to [NGC Documents](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/isaac-sim)).

   ```bash
   $ docker pull nvcr.io/nvidia/isaac-sim:4.2.0
   ```
1. Build docker image, replacing <your tag> with your desired tag:

   ```bash
   $ cd PATH/TO/GRUTOPIA/ROOT

   $ docker build -t grutopia:<your tag> .
   ```

1. Start docker container, replacing <your tag> with the above tag:

   ```bash
   $ cd PATH/TO/GRUTOPIA/ROOT

   $ export CACHE_ROOT=$HOME/docker  # set cache root path

   $ docker run --name grutopia -it --rm --gpus all --network host \
     -e "ACCEPT_EULA=Y" \
     -e "PRIVACY_CONSENT=Y" \
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


1. Download dataset to the specified directory.

   ```bash
   # run inside container
   $ python -m grutopia.download_assets
   ```

1. Verify the Installation.

   Run inside container:

   ```bash
   # run inside container
   $ python -m grutopia.demo.h1_locomotion  # start simulation
   ```

   If properly installed, observation from simulation will be displayed in the terminal every 100 steps, and you can access the Isaac Sim through Omniverse Streaming Client via `127.0.0.1`.


## Prepare Dataset Manually

If you prefer the dataset manually, or you only want to download a subset of the dataset, you can refer to the [dataset chapter](./how-to-use-grscenes.md) to find the links and content description.

After downloading the dataset, you should use the following command to tell GRUtopia where the assets locate (with GRUtopia installed):

```bash
$ python -m grutopia.set_assets_path
```

The command will prompt you to enter the path to the dataset.
