import carb
import numpy as np
import omni

from grutopia.core.util.interaction import BaseInteraction


class KeyboardController:
    def __init__(self):
        self.command = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    def read(self):
        """
        Subscribe to keyboard events
        """
        # subscribe to keyboard events
        app_window = omni.appwindow.get_default_app_window()  # noqa
        key_input = carb.input.acquire_input_interface()  # noqa
        key_input.subscribe_to_keyboard_events(app_window.get_keyboard(), self._sub_keyboard_event)

    def _sub_keyboard_event(self, event, *args, **kwargs):
        """subscribe to keyboard events, map to str"""
        if (
            event.type == carb.input.KeyboardEventType.KEY_PRESS
            or event.type == carb.input.KeyboardEventType.KEY_REPEAT
        ):
            if event.input == carb.input.KeyboardInput.W:
                self.command = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            if event.input == carb.input.KeyboardInput.S:
                self.command = np.array([0.0, 1.0, 0.0, 0.0, 0.0, 0.0])
            if event.input == carb.input.KeyboardInput.A:
                self.command = np.array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
            if event.input == carb.input.KeyboardInput.D:
                self.command = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0])
            if event.input == carb.input.KeyboardInput.Q:
                self.command = np.array([0.0, 0.0, 0.0, 0.0, 1.0, 0.0])
            if event.input == carb.input.KeyboardInput.E:
                self.command = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
        if event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            self.command = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])


@BaseInteraction.register('Keyboard')
class KeyboardInteraction(BaseInteraction):
    """Get keyboard input event(w, s, a, d)"""

    def __init__(self):
        super().__init__()
        self._type = 'Keyboard'
        self.controller = KeyboardController()

    def get_input(self) -> np.ndarray:
        """
        Read input of Keyboard.
        Returns:
            np.ndarray, len == 6.
        """
        self.controller.read()
        return self.controller.command
