from typing import Any, Dict


class ResponseModel:

    def __init__(self, code: int, msg: str, success: bool) -> None:
        self.data = None
        self.code = code
        self.msg = msg
        self.success = success

    def get_res_body(self) -> Dict[str, Any]:
        return {
            'code': self.code,
            'msg': self.msg,
            'success': self.success,
            'data': self.data,
        }


class SuccessModel(ResponseModel):

    def __init__(self, data: Any, msg: str = 'OK') -> None:
        super().__init__(0, msg, True)
        self.data = data


class ErrorModel(ResponseModel):

    def __init__(self, msg: str = 'got error') -> None:
        super().__init__(-1, msg, False)
