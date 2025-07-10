import os

import grutopia.core.object.isaacsim as isobj
import grutopia.core.robot.isaacsim as isrobot
from grutopia.core.runtime.task_runtime import TaskRuntime
from grutopia.core.scene import validate_scene_file
from grutopia.core.scene.scene import IScene


class IsaacsimScene(IScene):
    """IsaacSim's implementation on `IScene` class."""

    def __init__(self):
        from omni.isaac.core import World
        from omni.isaac.core.scenes import Scene

        self._scene: Scene = World.instance().scene

    def load(self, runtime: TaskRuntime):
        """See `IScene.load` for documentation."""
        usd_path = os.path.abspath(runtime.scene_asset_path)
        prim_path_root = f'World/env_{runtime.env.env_id}/scene'
        source, prim_path = validate_scene_file(usd_path, prim_path_root)

        from omni.isaac.core.utils.prims import create_prim

        position = [runtime.env.offset[idx] + i for idx, i in enumerate(runtime.scene_position)]
        scene_prim = create_prim(prim_path, usd_path=source, scale=runtime.scene_scale, translation=position)
        self.scene_prim = scene_prim

    def add(self, target: any):
        """See `IScene.add` for documentation."""
        if isinstance(target, isrobot.IsaacsimArticulation):
            if target.__class__ == isrobot.IsaacsimArticulation:
                self._scene.add(target._articulation)
            else:
                # TODO: Implement initialize method on IArticulation._articulation to make
                # 'self._scene._scene_registry.add_articulated_system' -> 'self._scene.add'
                self._scene._scene_registry.add_articulated_system(name=target.name, articulated_system=target)
        elif isinstance(target, isrobot.IsaacsimRigidBody):
            self._scene.add(target._rigid_prim)
        elif isinstance(target, isobj.IsaacsimDynamicCube):
            self._scene.add(target._cube)
        elif isinstance(target, isobj.IsaacsimUsdObject):
            self._scene.add(target._obj)
        else:
            # For instance of isaac-sim native classes
            self._scene.add(target)

    def remove(self, target: any, registry_only: bool = False):
        """See `IScene.remove` for documentation."""
        self._scene.remove_object(name=target, registry_only=registry_only)

    def object_exists(self, target: any) -> bool:
        """See `IScene.object_exists` for documentation."""
        return self._scene.object_exists(target)
