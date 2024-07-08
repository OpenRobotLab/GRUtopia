import typing

from pxr import Gf, Sdf, Usd, UsdGeom

from grutopia.core.scene.scene.util.type import get_xformop_precision, get_xformop_type
from grutopia.core.util import log


def add_usd_ref(source_stage: Usd.Stage, dest_stage: Usd.Stage, src_prim_path: str, dest_prim_path: str) -> None:
    """
    Add an opened usd into another usd as a reference
    set name in dest_prim_path

    Args:
        source_stage (Usd.Stage): source stage
        dest_stage (Usd.Stage): dest stage
        src_prim_path (str): source prim path
        dest_prim_path (str): dest prim path
    """
    src_root_layer = source_stage.GetRootLayer()
    log.debug(src_root_layer.identifier)
    source_prim = source_stage.GetPrimAtPath(src_prim_path)
    dest_prim = dest_stage.DefinePrim(dest_prim_path, source_prim.GetTypeName())
    dest_prim.GetReferences().AddReference(src_root_layer.identifier)
    dest_stage.GetRootLayer().Save()


def get_local_transform_xform(prim: Usd.Prim) -> typing.Tuple[Gf.Vec3d, Gf.Rotation, Gf.Vec3d]:
    """
    Get the local transformation of a prim using Xformable.

    Args:
        prim: The prim to calculate the local transformation.
    Returns:
        A tuple of:
        - Translation vector.
        - Rotation quaternion, i.e. 3d vector plus angle.
        - Scale vector.
    """
    xform = UsdGeom.Xformable(prim)
    local_transformation: Gf.Matrix4d = xform.GetLocalTransformation()
    translation: Gf.Vec3d = local_transformation.ExtractTranslation()
    rotation: Gf.Rotation = local_transformation.ExtractRotation()
    scale: Gf.Vec3d = Gf.Vec3d(*(v.GetLength() for v in local_transformation.ExtractRotationMatrix()))
    return translation, rotation, scale


def get_world_transform_xform(prim: Usd.Prim) -> typing.Tuple[Gf.Vec3d, Gf.Rotation, Gf.Vec3d]:
    """
    Get the local transformation of a prim using Xformable.

    Args:
        prim: The prim to calculate the world transformation.
    Returns:
        A tuple of:
        - Translation vector.
        - Rotation quaternion, i.e. 3d vector plus angle.
        - Scale vector.
    """
    xform = UsdGeom.Xformable(prim)
    time = Usd.TimeCode.Default()
    world_transform: Gf.Matrix4d = xform.ComputeLocalToWorldTransform(time)
    translation: Gf.Vec3d = world_transform.ExtractTranslation()
    rotation: Gf.Rotation = world_transform.ExtractRotation()
    scale: Gf.Vec3d = Gf.Vec3d(*(v.GetLength() for v in world_transform.ExtractRotationMatrix()))
    return translation, rotation, scale


def create_new_usd(new_usd_path: str, default_prim_name: str, default_axis: str = None) -> Usd.Stage:
    """
    Create a new usd

    Args:
        new_usd_path (str): where to place this new usd
        default_prim_name (str): default prim name (root prim path)
        default_axis (str): default axis for new usd
    """
    stage: Usd.Stage = Usd.Stage.CreateNew(new_usd_path)
    default_prim: Usd.Prim = UsdGeom.Xform.Define(stage, Sdf.Path('/' + default_prim_name)).GetPrim()
    _set_default_prim(stage, default_prim)
    _set_up_axis(stage, default_axis)
    stage.GetRootLayer().Save()
    return stage


def _set_up_axis(stage: Usd.Stage, axis_str: str = None) -> None:
    """
    Set default axis for a stage

    Args:
        stage (Usd.Stage): objects stage
        axis_str (str, optional): axis str, 'y' or 'z', set 'z' if None. Defaults to None.
    """
    if axis_str == 'y' or axis_str == 'Y':
        axis: UsdGeom.Tokens = UsdGeom.Tokens.y
    else:
        axis: UsdGeom.Tokens = UsdGeom.Tokens.z
    UsdGeom.SetStageUpAxis(stage, axis)


def _set_default_prim(stage: Usd.Stage, prim: Usd.Prim) -> None:
    """
    Set default prim for a stage

    Args:
        stage (Usd.Stage): objects stage
        prim (Usd.Prim): prim in this stage
    """
    stage.SetDefaultPrim(prim)


def compute_bbox(prim: Usd.Prim) -> Gf.Range3d:
    """
    Compute Bounding Box using ComputeWorldBound at UsdGeom.Imageable

    Args:
        prim: A prim to compute the bounding box.
    Returns:
        A range (i.e. bounding box)
    """
    imageable: UsdGeom.Imageable = UsdGeom.Imageable(prim)
    time = Usd.TimeCode.Default()
    bound = imageable.ComputeWorldBound(time, UsdGeom.Tokens.default_)
    bound_range = bound.ComputeAlignedBox()
    return bound_range


def delete_prim_in_stage(stage: Usd.Stage, prim: Usd.Prim) -> None:
    """
    Delete a prim in stage

    Args:
        stage (Usd.Stage): objects stage
        prim (Usd.Prim): prim to be deleted
    """
    stage.RemovePrim(prim.GetPrimPath())


def set_xform_of_prim(prim: Usd.Prim, xform_op: str, set_valve: typing.Any) -> None:
    """
    Set xform data of a prim with new data

    Args:
        prim (Usd.Prim): objects prim
        xform_op (str): which op to be set
        set_valve (typing.Any): new data to be set, could be np.array
    """
    stage = prim.GetStage()
    op_list = prim.GetAttribute('xformOpOrder').Get()
    s = None
    for i in op_list:
        if xform_op == i:
            log.debug(prim.GetAttribute(i))
            s = prim.GetAttribute(i)
    trans = s.Get()
    trans_value = set_valve
    data_class = type(trans)
    time_code = Usd.TimeCode.Default()
    new_data = data_class(*trans_value)
    s.Set(new_data, time_code)
    stage.Save()


def delete_xform_of_prim(prim: Usd.Prim, xform_op: str) -> None:
    """
    Delete xform data of a prim

    Args:
        prim (Usd.Prim): objects prim
        xform_op (str): which op to be deleted
    """
    stage = prim.GetStage()
    if prim.HasAttribute(xform_op):
        # Clear the attribute from the Prim
        prim.GetAttribute(xform_op).Clear()
    stage.Save()


def add_xform_of_prim(prim: Usd.Prim, xform_op: str, set_valve: typing.Any) -> None:
    """
    Add xform data of a prim with new data

    Args:
        prim (Usd.Prim): objects prim
        xform_op (str): which op to be set
        set_valve (typing.Any): new data to be set, could be Gf.Vec3d, Gf.Rotation
    """
    stage = prim.GetStage()
    attribute_name = xform_op
    attribute_value = set_valve
    opType = get_xformop_type(xform_op)
    precision = get_xformop_precision('float')
    attribute = UsdGeom.Xformable(prim).AddXformOp(opType, precision)
    if attribute:
        attribute.Set(attribute_value)
        # log.debug(f"Attribute {attribute_name} has been set to {attribute_value}.")
    else:
        log.debug(f'Failed to create attribute named {attribute_name}.')
    stage.Save()


def add_xform_of_prim_old(prim: Usd.Prim, xform_op: str, set_valve: typing.Any) -> None:
    """
    Add xform data of a prim with new data

    Args:
        prim (Usd.Prim): objects prim
        xform_op (str): which op to be set
        set_valve (typing.Any): new data to be set, could be Gf.Vec3d, Gf.Rotation
    """
    stage = prim.GetStage()
    attribute_name = xform_op
    attribute_value = set_valve
    if '3' in type(set_valve).__name__:
        attribute_type = Sdf.ValueTypeNames.Float3
    else:
        attribute_type = Sdf.ValueTypeNames.Float
    attribute = prim.CreateAttribute(attribute_name, attribute_type)
    if attribute:
        attribute.Set(attribute_value)
        # log.debug(f"Attribute {attribute_name} has been set to {attribute_value}.")
    else:
        log.debug(f'Failed to create attribute named {attribute_name}.')
    stage.Save()
