# Teleoperating with Mocap

This tutorial will guide you how to teleoperate with hand gesture-based motion capture (Mocap). It takes Franka robotic arm as example to give an instruction for the process of setting up and controlling the Franka robotic arm using Mocap. By following this guide, you will be able to complete the [`GRUtopia/grutopia/demo/franka_manipulation_mocap_teleop.py`](https://github.com/OpenRobotLab/GRUtopia/blob/main/grutopia/demo/franka_manipulation_mocap_teleop.py) task, enabling flexible control of the arm’s movement, rotation, gripper actions, and even the camera position in the simulated environment. Let's get started by following the steps below.

## 1. Implementation Guide

### Step 1: Set Up a Real-Time Data Stream Server with an RGB Camera

First, create a new conda environment, and install all necessary dependencies:

```bash
$ pip install opencv-python flask
```

Second, you'll need to set up a real-time image data stream from an RGB camera. The provided script, [`GRUtopia/toolkits/mocap/rgb_camera_server.py`](https://github.com/OpenRobotLab/GRUtopia/blob/main/toolkits/mocap/rgb_camera_server.py), can be used as a reference to create a local streaming server. The command to start the server is:

```bash
$ python rgb_camera_server.py
```

After running the script, make sure to note the server’s URL. If you're using the sample code and running it locally, the URL (using the loopback address as an example) should be: `http://127.0.0.1:5000/video`.

<video width="720" height="405" controls>
    <source src="../../_static/video/mocap_rgb_camera_server.webm" type="video/webm">
</video>

> **Note:**
> The provided code is a reference and may need adjustments based on your specific setup. In the simplest case, you can use your laptop's built-in camera to capture the real-time data.


### Step 2: Set Up a Real-Time Hand Gesture Recognition Server with Hamer

> In addition to the Hamer environment, the following Python packages are required: `pip install OneEuroFilter flask`

Next, you'll set up the hand gesture recognition system using Hamer. You can find the relevant GitHub repository for Hamer [here](https://github.com/geopavlakos/hamer). After setting up the Hamer environment, place [`GRUtopia/toolkits/mocap/hamer_real_time.py`](https://github.com/OpenRobotLab/GRUtopia/blob/main/toolkits/mocap/hamer_real_time.py) and [`hamer_real_time_server.py`](https://github.com/OpenRobotLab/GRUtopia/blob/main/toolkits/mocap/hamer_real_time_server.py) into the `hamer/` directory. The command to start the server is:

```bash
$ python hamer_real_time_server.py --video_url {{video_url}}
```

The `video_url` parameter should correspond to the URL generated in Step 1. For example, if you're using the URL from Step 1, the command should be:

```bash
$ python hamer_real_time_server.py --video_url http://127.0.0.1:5000/video
```

Once the server is running successfully, make a note of the Hamer server URL (e.g., `http://127.0.0.1:5001` as shown in the example code).

### Step 3: Start the Task in GRUtopia

Now, update the `mocap_url` parameter in [`GRUtopia/grutopia/demo/franka_manipulation_mocap_teleop.py`](https://github.com/OpenRobotLab/GRUtopia/blob/main/grutopia/demo/franka_manipulation_mocap_teleop.py) with the Hamer server URL obtained in Step 2 (e.g., `http://127.0.0.1:5001`). After that, start the task by running the following command:

```
python -m grutopia.demo.franka_manipulation_mocap_teleop
```

After the Isaac Sim interface finishes loading, move your right hand in front of the RGB camera (from Step 1), and the Franka arm will follow your hand movements.

<video width="720" height="405" controls>
    <source src="../../_static/video/mocap_start.webm" type="video/webm">
</video>

---

## 2. Operation Logic

Once the system is up and running, you can use hand gestures to control various functions of the Franka robotic arm. Below are the control logic and gestures used for different actions:

### 2.1 Control the virtual Camera's Third-Person View Rotation

- **Gesture:** Pinch the left thumb and index finger for about 0.5 seconds, then move the fingers in any direction (up/down/left/right/forward/backward) to control the virtual camera's movement (in the Isaac Sim GUI, select the camera `lh_controlled_camera` from the `Camera` menu under `Perspective`).

<video width="720" height="405" controls>
    <source src="../../_static/video/mocap_camera_control.webm" type="video/webm">
</video>

### 2.2 Control the Gripper Movement

- **Gesture:** Without pinching the left thumb and pinky, move the right hand in any direction (up/down/left/right/forward/backward) relative to the virtual camera's view to control the movement of the robotic arm's gripper.

- **Explanation:**
  - **View:** The gripper's movement will follow the perspective of the virtual camera (in the Isaac Sim GUI, select the virtual camera named `panda_hand_camera` from the `Camera` menu under `Perspective`). When your fingers are naturally extended and open, the gripper will follow the direction of your hand (i.e., if you move your hand in the direction of your fingers, the gripper will follow).
  - **Reset Right Hand’s Starting Position:** To prevent the right hand from moving out of the virtual camera's view, pinch the left thumb and pinky for about 0.1 seconds to reset the starting position. Once you release the pinch, the new position of the right hand will be set as the origin for controlling the arm.
  - **Adjust Human-Robot Movement Ratio:** To improve precision during delicate tasks, pinch the left thumb and ring finger for 0.1 seconds. This will halve the movement ratio between your hand and the robot, changing the original `1:1` ratio to `1:0.5` for finer control.

### 2.3 Control the Gripper's Orientation

- **Gesture:** Without pinching the left thumb and middle finger, move the right hand to rotate the gripper's orientation in alignment with the world coordinate system.

- **Explanation:**
  - **Fix the Gripper's Orientation:** To prevent unintended rotation of the gripper due to camera angle changes, pinch the left thumb and middle finger for about 0.1 seconds. During this time, the gripper's orientation will remain fixed, even if the right hand rotates.

### 2.4 Gripper Grab and Release

- **Gesture:** Pinch the right thumb and index finger to close the gripper, and release the pinch to open it.

<video width="720" height="405" controls>
    <source src="../../_static/video/mocap_franka_control.webm" type="video/webm">
</video>

---

### Additional Notes

- Make sure the video stream server (Step 1) is running and accessible at the specified URL. If you're using an external camera, double-check the camera settings.
- Ensure that only one person's hands are visible in the frame; multiple hands may cause unexpected behavior.
