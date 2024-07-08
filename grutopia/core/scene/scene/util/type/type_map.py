from pxr import Gf, UsdGeom

dtype_map = {
    'double2': Gf.Vec2d,
    'double3': Gf.Vec3d,
    'double4': Gf.Vec4d,
    'float2': Gf.Vec2f,
    'float3': Gf.Vec3f,
    'float4': Gf.Vec4f,
    'half2': Gf.Vec2h,
    'half3': Gf.Vec3h,
    'half4': Gf.Vec4h,
    'int2': Gf.Vec2i,
    'int3': Gf.Vec3i,
    'int4': Gf.Vec4i,
    'matrix2d': Gf.Matrix2d,
    'matrix3d': Gf.Matrix3d,
    'matrix4d': Gf.Matrix4d,
    'quatd': Gf.Quatd,
    'quatf': Gf.Quatf,
    'quath': Gf.Quath,
}


def get_xformop_precision(precision: str) -> UsdGeom.XformOp.Precision:
    if 'double' in precision:
        return UsdGeom.XformOp.PrecisionDouble
    elif 'float' in precision:
        return UsdGeom.XformOp.PrecisionFloat
    else:
        return UsdGeom.XformOp.PrecisionHalf


def get_xformop_type(x_type: str) -> UsdGeom.XformOp.Type:
    if ':' in x_type:
        x_type = x_type.split(':')[1]
    return UsdGeom.XformOp.Type.GetValueFromName('Type' + x_type[0].capitalize() + x_type[1:])
