# Installation

## Prerequisites

- OS: Ubuntu 20.04/22.04
- RAM: 32GB+
- GPU: NVIDIA RTX 2070+ (must with RTX cores)
- NVIDIA Driver: 535.216.01+

> For complete requirements, please see [Isaac Sim's Requirements](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/requirements.html).
>
> GRUtopia is built upon NVIDIA's [Omniverse](https://www.nvidia.com/en-us/omniverse/) and [Isaac Sim](https://developer.nvidia.com/isaac-sim) platforms, inheriting their dependencies. GRUtopia 2.0 specifically requires **Isaac Sim 4.2.0**. To ensure optimal performance and avoid any potential issues, it is essential to use this version rather than any other releases, such as 4.5.0 or 4.1.0.

## Installation

Three ways of installation are provided:

- [Install from source (Linux)](#install-from-source-linux): recommended for users who wants to thoroughly explore GRUtopia with Isaac Sim as a GUI application on Linux workstation with a GPU.
- [Install from PyPI (Linux)](#install-from-pypi-linux): recommended for users who wants to use GRUtopia as a handy toolbox with Isaac Sim as a GUI application on Linux workstation with a GPU.
- [Install with Docker (Linux)](#install-with-docker-linux): recommended for deployment on remote servers or the Cloud using a Docker container.

See more: [Differences Between Workstation And Docker](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_faq.html#isaac-sim-setup-differences).

Windows support is in our roadmap. Contributions are welcome!


### Install from source (Linux)

Before proceeding with the installation, ensure that you have [Isaac Sim 4.2.0](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_workstation.html) and [Conda](https://conda.io/projects/conda/en/latest/user-guide/install/index.html) installed.

1. Clone the GRUtopia repository with [git](https://git-scm.com).
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

### Install from PyPI (Linux)

Before proceeding with the installation, ensure that you have [Isaac Sim 4.2.0](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_workstation.html) and [Conda](https://conda.io/projects/conda/en/latest/user-guide/install/index.html) installed.

1. Create conda env with `python=3.10` specified.
    ```bash
   $ conda create -n <env> python=3.10
   ```
2. Install GRUtopia through pip.

   **NOTE**: Ensure you have [git](https://git-scm.com) installed.

   ```bash
   $ conda activate <env>
   $ pip install grutopia
   ```
3. Configure the conda environment.

   ```bash
   $ python -m grutopia.setup_conda_pypi

   $ conda deactivate && conda activate <env>
   ```

### Install with Docker (Linux)

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
   $ xhost +local:root # Allow the container to access the display

   $ cd PATH/TO/GRUTOPIA/ROOT

   $ docker run --name grutopia -it --rm --gpus all --network host \
     -e "ACCEPT_EULA=Y" \
     -e "PRIVACY_CONSENT=Y" \
     -e "DISPLAY=${DISPLAY}" \
     -v /tmp/.X11-unix/:/tmp/.X11-unix \
     -v ${PWD}:/isaac-sim/GRUtopia \
     -v ${HOME}/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
     -v ${HOME}/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
     -v ${HOME}/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
     -v ${HOME}/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
     -v ${HOME}/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
     -v ${HOME}/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
     -v ${HOME}/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
     -v ${HOME}/docker/isaac-sim/documents:/root/Documents:rw \
     grutopia:<your tag>
   ```

   You are now ready to use GRUtopia in this container.

   **NOTE**: If you are using a remote server without display, you can use the [Omniverse Streaming Client](https://docs.omniverse.nvidia.com/extensions/latest/ext_livestream/native.html) to stream the simulation UI.

## Prepare Assets

```{note}
📝First of all you **MUST** complete the [User Agreement for GRScenes-100 Dataset Access](https://docs.google.com/forms/d/e/1FAIpQLSccX4pMb57eZbjXpH12Jz6WUBmCfeyc2t0s98k_u4Z-GD3Org/viewform?fbzx=8256642192244696391).
```

Then you can one of the following methods to get the assets:

- Download the assets automatically with [GRUtopia](#installation) installed:

  ```shell
  $ python -m grutopia.download_assets
  ```

  During the script execution, you can choose to download full assets (~80GB) or a minimum set (~500MB), and you will be asked to specify the local path to store the downloaded assets.

  > **NOTE**: If GRUtopia is installed with Docker, We recommend downloading the assets to a location under `/isaac-sim/GRUtopia/` in container (which is mounted from a host path) so that it can be retained across container recreations.

- Download the assets manually from [HuggingFace](https://huggingface.co/datasets/OpenRobotLab/GRScenes)/[ModelScope](https://www.modelscope.cn/datasets/Shanghai_AI_Laboratory/GRScenes/summary)/[OpenDataLab](https://openxlab.org.cn/datasets/OpenRobotLab/GRScenes), and then use the following command to tell GRUtopia where the assets locate if you are meant to use it with GRUtopia:

  ```shell
  $ python -m grutopia.set_assets_path
  ```

## Verify Installation

```shell
$ python -m grutopia.demo.h1_locomotion  # start simulation
```

If properly installed, Isaac Sim GUI window would pop up and you can see a humanoid robot (Unitree H1) walking following a pre-defined trajectory in Isaac Sim.

<video width="720" height="405" controls>
    <source src="../../../_static/video/h1_locomotion.webm" type="video/webm">
</video>

> **NOTE**: A slowdown is expected during first execution.
> Isaac sim requires some one-time startup setup the first time you start it.
> The process could take up to 5 minutes. This is expected behavior, and should only occur once!
