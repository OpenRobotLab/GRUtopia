from grutopia.core.scene.scene.util.type.type_map import (
    dtype_map,
    get_xformop_precision,
    get_xformop_type,
)


def get_usd_data_type(dtype: str):
    return dtype_map[dtype]
