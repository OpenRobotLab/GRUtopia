# code of class `Tool` and `PythonREPL` are adapted from https://github.com/xingyaoww/code-act/blob/main/mint/
import json
import signal
from abc import ABC, abstractmethod
from contextlib import contextmanager
from typing import Any, List, Mapping, Optional, Tuple

import numpy as np
from IPython.core.interactiveshell import InteractiveShell
from IPython.utils import io
from pydantic import BaseModel


class Tool(ABC):
    """Abstract class for a tool."""

    name: str
    signature: str
    description: str

    @abstractmethod
    def __call__(self, *args: Any, **kwds: Any) -> str:
        """Execute the tool with the given args and return the output."""
        # execute tool with arbitrary args
        pass

    def reset(self) -> None:
        """Reset the tool to its initial state."""
        pass


class PythonREPL(Tool):
    """A tool for running python code in a REPL."""

    name = 'PythonREPL'
    # This PythonREPL is not used by the environment; It is THE ENVIRONMENT.
    signature = 'NOT_USED'
    description = 'NOT_USED'

    def __init__(
        self,
        user_ns: Mapping[str, Any],
        timeout: int = 30,
    ) -> None:
        super().__init__()
        self.user_ns = user_ns
        self.timeout = timeout
        self.reset()

    @contextmanager
    def time_limit(self, seconds):

        def signal_handler(signum, frame):
            raise TimeoutError(f'Timed out after {seconds} seconds.')

        signal.signal(signal.SIGALRM, signal_handler)
        signal.alarm(seconds)
        try:
            yield
        finally:
            signal.alarm(0)  # Disable the alarm

    def reset(self) -> None:
        InteractiveShell.clear_instance()
        self.shell = InteractiveShell.instance(
            # NOTE: shallow copy is needed to avoid
            # shell modifying the original user_ns dict
            user_ns=dict(self.user_ns),
            colors='NoColor',
        )

    def set_args(self, user_ns: Mapping[str, Any]) -> None:
        self.user_ns.update(user_ns)
        self.shell = InteractiveShell.instance(
            user_ns=dict(self.user_ns),
            colors='NoColor',
        )

    def __call__(self, query: str) -> str:
        """Use the tool and return observation"""
        with io.capture_output() as captured:
            _ = self.shell.run_cell(query, store_history=True)
        output = captured.stdout

        if output == '':
            output = '[Executed Successfully with No Output]'

        if len(output) > 10000:
            output = output[:10000] + '...\n[Output Truncated]'

        return output


class Object(BaseModel):
    id: str
    category: str
    center: np.ndarray
    size: np.ndarray
    caption: str
    location: str
    nearby_objects: List[Tuple[str, str]]

    class Config:
        arbitrary_types_allowed = True


class ObjectInView(BaseModel):
    id: str
    center: np.ndarray
    size: np.ndarray

    class Config:
        arbitrary_types_allowed = True


class Env:

    def __init__(self, scene_data_path: str):
        with open(scene_data_path) as f:
            self.scene_data = json.load(f)
        self.objects = []
        for id, obj_info in self.scene_data.items():
            position = np.array(obj_info['position'])
            size = np.array(obj_info['max_points']) - np.array(obj_info['min_points'])
            nearby_objects = [(nearby_obj_id, spatial_relation[0])
                              for nearby_obj_id, spatial_relation in obj_info['nearby_objects'].items()]
            obj = Object(id=id,
                         category=obj_info['category'],
                         center=position,
                         size=size,
                         caption=obj_info['caption'],
                         location=obj_info['room'],
                         nearby_objects=nearby_objects)
            self.objects.append(obj)
        # yapf: disable
        self.shell = PythonREPL(user_ns={
            'all_objects': self.objects,
            'all_objects_in_view': [],
            'current_position': np.array([0, 0, 0]),
            'current_orientation': np.array([0, 0, 0, 0]),
            'Object': Object,
            'ObjectInView': ObjectInView,
            'Optional': Optional,
            'np': np
        }, timeout=30)
        # yapf: enable

    def reset(self) -> None:
        self.shell.reset()

    def __call__(self, code: str) -> str:
        return self.shell(code)
