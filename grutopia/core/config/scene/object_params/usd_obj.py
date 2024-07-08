from pydantic import BaseModel


class UsdObj(BaseModel):
    usd_path: str
