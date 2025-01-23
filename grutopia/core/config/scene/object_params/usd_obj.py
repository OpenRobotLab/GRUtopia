from pydantic import BaseModel


class UsdObj(BaseModel):
    """
    Represents a USD object with a specific file path.

    This class extends the BaseModel to include a 'usd_path' attribute, which stores the file path to a USD (Universal Scene Description) file. The class provides a foundation for working with USD data within the application.

    TODO: Add more param here.

    Attributes:
        usd_path (str): The file system path to the USD file associated with this object.
    """

    usd_path: str
