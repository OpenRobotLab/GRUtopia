# Teleoperating with VisionPro

## Prerequisites

### 1. Generate a Self-Signed SSL Certificate
This is used to set up the server for HTTPS communication with VisionPro. VisionPro needs to be connected to the server network (generally, being on the same local network/router is sufficient). The certificate can be generated using mkcert, which can be installed via Homebrew. The specific command for generating the certificate is as follows:
```
# Install Homebrew on Linux
$ /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
$ (echo; echo 'eval "$(/home/linuxbrew/.linuxbrew/bin/brew shellenv)"') >> ~/.bashrc
$ source ~/.bashrc

# Install mkcert
$ sudo apt-get install build-essential libnss3-tools
$ brew install mkcert

# Generate certs for local address.
$ ip addr | grep inet  # Find the IP address Vision Pro can access, assume 192.168.100.101
$ cd GRUtopia && mkdir mkcert && cd mkcert
$ mkcert -install  # Generate the local CA and install it in the system trust store.
$ mkcert -cert-file cert.pem -key-file key.pem 192.168.100.101 localhost 127.0.0.1  # Replace 192.168.100.101 with your IP addr

# Get the location of local CA cert.
$ mkcert -CAROOT
$ ls $(mkcert -CAROOT)
```

### 2. Transfer the CA Certificate to VisionPro and Trust It
To transfer the certificate to VisionPro, you can start a server and then access it from VisionPro to download the certificate directly.
```
$ cd $(mkcert -CAROOT) && python -m http.server
```
1. Open Settings and install the downloaded profile
2. Settings > General > About > Certificate Trust Settings. Under "Enable full trust for root certificates", turn on trust for the certificate.
3. Settings > Apps > Safari > Advanced > Feature Flags > Enable "WebXR Related Features"

### 3. Prepare the Conda Environment
- if you only need to use VisionPro to obtain pose data, you can directly run the following command in the Isaac Sim Conda environment:
```
$ pip install -r requirements/runtime.txt
```
- If you need to control GR1, you will need to create a new Conda environment and then run:
```
$ conda create -n sim-teleop python=3.10
$ conda activate sim-teleop
$ pip install -r requirements/teleop.txt
```

## Running Instructions
This section explains how to obtain the pose data collected by VisionPro during runtime.

In this document, we will use the GR1 as an example to demonstrate how to utilize VisionPro for teleoperating it. For a complete example of GR1 teleoperation, please refer to the next section.

In the demo script, you need to specify the server certificate and key generated previously using mkcert.
```PYTHON
from grutopia.core.env import Env
from grutopia.core.runtime import SimulatorRuntime
from grutopia.core.util.container import is_in_container
from grutopia_extension import import_extensions

# Run with single inference task
file_path = './GRUtopia/demo/configs/gr1t2_teleop.yaml'

headless = False
webrtc = False

if is_in_container():
    headless = True
    webrtc = True

sim_runtime = SimulatorRuntime(config_path=file_path, headless=headless, webrtc=webrtc)

from grutopia_extension.interactions.visionpro.visionpro import VuerTeleop

import_extensions()
# import custom extensions here.

env = Env(sim_runtime)

import numpy as np

actions = {}

teleop = VuerTeleop(cert_file='./GRUtopia/mkcert/cert.pem',
                    key_file='./GRUtopia/mkcert/key.pem',
                    resolution=(720, 1280))

i = 0
while env.simulation_app.is_running():
    i += 1

    # Get head and wrist pose from vision pro.
    # shapes:
    # - head_mat: (4, 4)
    # - left_wrist_mat: (4, 4)
    # - right_wrist_mat: (4, 4)
    # - left_hand_mat: (25, 3)
    # - right_hand_mat: (25, 3)
    head_mat, left_wrist_mat, right_wrist_mat, left_hand_mat, right_hand_mat, begin_move = teleop.step()

    # TODO: control and get vision from camera.

    # Send RGB vision frame to vision pro.
    frame = (frame * 255).astype(np.uint8)
    np.copyto(teleop.img_array, frame)

teleop.cleanup()
env.simulation_app.close()
```
Then open Safari on VisionPro and navigate to:
 https://192.168.8.102:8012?ws=wss://192.168.8.102:8012 (Replace 192.168.8.102 with your own IP address.)

The pose data obtained by VisionPro is represented as a 4x4 transformation matrix.

The `hand_mat` is primarily used for retargeting the dexterous hand and has a shape of (25, 3).

The following are GR1 Camera Settings:
```PYTHON
    camera_right = CameraCfg(
        height=360*2,
        width=640*2,
        offset=CameraCfg.OffsetCfg(pos=(0.15, -0.06, 0.1), rot=(0.92388, 0.0, 0.38268, 0.0), convention='world'),
        prim_path='/World/Robot/head_yaw_link/CameraRight',
        update_period=1/60,
        data_types=['rgb'],#,'depth'],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=1.5, focus_distance=10.0, horizontal_aperture=2, clipping_range=(0.05, 15)
        ),
    )

    camera_left = CameraCfg(
        height=360*2,
        width=640*2,
        offset=CameraCfg.OffsetCfg(pos=(0.15, 0.06, 0.1), rot=(0.92388, 0.0, 0.38268, 0.0), convention='world'), # wxyz in isaacsim
        prim_path='/World/Robot/head_yaw_link/CameraLeft', # TODO fit in actual offset of camera and head
        update_period=1/60,
        data_types=['rgb'],#,'depth'],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=1.5, focus_distance=10.0, horizontal_aperture=2, clipping_range=(0.05, 15)
        ),
    )
```

## Example
For a complete example demo, please refer to [this link](https://github.com/OpenRobotLab/GRUtopia/demo/gr1t2_teleop.py)

The example utilizes robot and object assets, which should be extracted into the following directories:
- assets/robots
- assets/objects

The server certificates can be found at:
- GRUtopia/mkcert/cert.pem
- GRUtopia/mkcert/key.pem

To run this example, you need to start the solver process for GR1 first, followed by the Isaac Sim process:
```
$ conda create -n sim-teleop python=3.10
$ conda activate sim-teleop
$ pip install -r requirements/teleop.txt
$ cd GRUtopia/grutopia_extension/controllers && conda run --no-capture-output -n sim-teleop python gr1t2_teleop.py
```
if you see the message "waiting for teleop action...", it indicates that the solver process has started successfully. Then, in the Isaac Sim Conda environment, run the following command:
```
$ python GRUtopia/demo/gr1t2_teleop.py
```
Once the simulation begins, open Safari on VisionPro and navigate to:
https://192.168.8.102:8012?ws=wss://192.168.8.102:8012(Replace 192.168.8.102 with your own IP address.)
Select "Enter VR" to begin operations.
