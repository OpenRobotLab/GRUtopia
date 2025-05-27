# Layout Edit with Mocap

This tutorial will guide you on how to remotely operate gesture-based motion capture (Mocap). Using layout editing as an example, it provides a description of the process of building and editing layouts using motion capture technology. By following this guide, you will be able to complete the [`GRUtopia/toolkits/layout_edit/layout_edit.py`](https://github.com/OpenRobotLab/GRUtopia/blob/main/toolkits/layout_edit/layout_edit.py) task, allowing you to flexibly edit the contents of the layout, even adjusting the camera position in the simulation environment. Let's start with the steps below.

## 1. Implementation Guide
Please refer to the `Implementation Guide` Step1 and Step2 content [here](teleoperating-with-mocap.md#implementation-guide) for the process.

Then start the task in GRUtopia:

Update the `mocap_url` parameter in [`GRUtopia/toolkits/layout_edit/layout_edit.py`](https://github.com/OpenRobotLab/GRUtopia/blob/main/grutopia/toolkits/layout_edit/layout_edit.py) with the Hamer server URL obtained in Step 2 (e.g., `http://127.0.0.1:5001`). After that, start the task by running the following command:

```
python -m toolkits.layout_edit.layout_edit
```

After the Isaac Sim interface finishes loading, perform layout edit according to `2. Operation Logic`.

---

## 2. Operation Logic

Once the system is up and running, you can use gestures to edit various objects in the layout, and finally save the edited layout. The following is the control logic and gestures for different actions:


### 2.1 View Switching

Adjust the third-person camera to align with our perspective to view the current layout.

#### Operation

- In the Isaac Sim GUI, select the virtual camera named `layout_controlled_camera` from the `Camera` menu under `Perspective`

### 2.2 Dynamic Loading and Unloading of Both Hands

To ensure that gestures in the virtual scenario are loaded as needed and to avoid the issue of gestures having no place to be when initially entering the scenario, gestures need to be dynamically loaded based on requirements. Additionally, in a realistic virtual scenario, there are no hands by default, so gestures should be dynamically loaded according to the need and dynamically removed after the operation is completed.
#### Operation
- Loading hand：

  - **Left-hand loading:** By pinching the left thumb and ring finger together for about 0.5 seconds, the gesture in the virtual scene is loaded.

  - **Right-hand loading:** By pinching the right thumb and ring finger together for about 0.5 seconds, the gesture in the virtual scene is loaded.

- Unloading hand:

    - **Both hands unloading:** By pinching the thumb and middle finger of the right hand for approximately 0.5 seconds, you can directly clear any gestures in the current scene.

### 2.3 Layout edit

This guide explains how to use gesture-based motion capture (Mocap) to edit layouts. In a virtual scene, gestures in the layout will follow the actual hand movements (up/down/left/right/rotate), allowing you to edit the content of the layout by following the actual gesture of grabbing.

#### Operation

  - **Editing the scene with the right hand:** Pinch the thumb and index finger of the right hand together to bind the object closest to the front of the hand. Then, the object changes according to the position and rotation angle of the hand.

  - **Editing the scene with the left hand:** Pinch the thumb and index finger of the left hand together to bind the object closest to the front of the hand. Then, the object changes according to the position and rotation angle of the hand.

#### Explain

  - **Reset Right Hand’s Starting Position:** To prevent the right hand from moving out of the camera's view, pinch the thumb and pinky finger of the left hand together for approximately 0.5 seconds to reset the position of the right hand. Once the pinch is released, set the current position of the right hand as the position of the hand in the virtual scene.

  - **Reset Left Hand’s Starting Position:** To prevent the left hand from moving out of the camera's view, pinch the thumb and pinky finger of the right hand together for approximately 0.5 seconds to reset the position of the left hand. Once the pinch is released, set the current position of the left hand as the position of the hand in the virtual scene.


### 2.4 Adjusting View

To ensure the authenticity of the interaction process, the camera represents the human perspective, allowing for free movement and rotation of the viewpoint.

#### Operation

- Pinch the thumb and middle finger of the left hand together for approximately 0.5 seconds, then move the gesture in any direction (up/down/left/right/forward/backward) to control the movement of the virtual camera.

### 2.5 Saving layout

After editing the scene, save the scene as needed.

#### Operation

- Pinch the thumb and middle finger of the right hand together for approximately 0.5 seconds to delete the loaded gesture in the virtual scene. Then, the current state of the scene is automatically saved to the `/GRUtopia/out` directory.

<video width="720" height="405" controls>
    <source src="../../_static/video/mocap_layout_edit.webm" type="video/webm">
</video>

---

### Additional Notes

- Make sure the video stream server (Step 1) is running and accessible at the specified URL. If you're using an external camera, double-check the camera settings.
- Ensure that only one person's hands are visible in the frame; multiple hands may cause unexpected behavior.
