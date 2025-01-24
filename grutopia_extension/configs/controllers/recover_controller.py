from typing import Optional

from grutopia.core.config.robot import ControllerModel


class RecoverControllerCfg(ControllerModel):

    type: Optional[str] = 'RecoverController'
    recover_height: float
