# Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

import os
from typing import Any, List, Optional

import numpy as np
import torch
from modules.vlm.server_wrapper import (
    ServerMixin,
    bool_arr_to_str,
    host_model,
    send_request,
    str_to_bool_arr,
    str_to_image,
)

try:
    from mobile_sam import SamPredictor, sam_model_registry
except ModuleNotFoundError:
    print('Could not import mobile_sam. This is OK if you are only using the client.')


class MobileSAM:
    def __init__(
        self,
        sam_checkpoint: str,
        model_type: str = 'vit_t',
        device: Optional[Any] = None,
    ) -> None:
        if device is None:
            device = torch.device('cuda') if torch.cuda.is_available() else 'cpu'
        self.device = device

        mobile_sam = sam_model_registry[model_type](checkpoint=sam_checkpoint)
        mobile_sam.to(device=device)
        mobile_sam.eval()
        self.predictor = SamPredictor(mobile_sam)

    def segment_bbox(self, image: np.ndarray, bbox: List[int]) -> np.ndarray:
        """Segments the object in the given bounding box from the image.

        Args:
            image (numpy.ndarray): The input image as a numpy array.
            bbox (List[int]): The bounding box as a numpy array in the
                format [x1, y1, x2, y2].

        Returns:
            np.ndarray: The segmented object as a numpy array (boolean mask). The mask
                is the same size as the bbox, cropped out of the image.

        """
        with torch.inference_mode():
            self.predictor.set_image(image)
            masks, _, _ = self.predictor.predict(box=np.array(bbox), multimask_output=False)

        return masks[0]


class MobileSAMClient:
    def __init__(self, port: int = 12183):
        self.url = f'http://localhost:{port}/mobile_sam'

    def segment_bbox(self, image: np.ndarray, bbox: List[int]) -> np.ndarray:
        response = send_request(self.url, image=image, bbox=bbox)
        cropped_mask_str = response['cropped_mask']
        cropped_mask = str_to_bool_arr(cropped_mask_str, shape=tuple(image.shape[:2]))

        return cropped_mask


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=12183)
    args = parser.parse_args()

    print('Loading model...')

    class MobileSAMServer(ServerMixin, MobileSAM):
        def process_payload(self, payload: dict) -> dict:
            image = str_to_image(payload['image'])
            cropped_mask = self.segment_bbox(image, payload['bbox'])
            cropped_mask_str = bool_arr_to_str(cropped_mask)
            return {'cropped_mask': cropped_mask_str}

    mobile_sam = MobileSAMServer(sam_checkpoint=os.environ.get('MOBILE_SAM_CHECKPOINT', 'data/mobile_sam.pt'))
    print('Model loaded!')
    print(f'Hosting on port {args.port}...')
    host_model(mobile_sam, name='mobile_sam', port=args.port)
