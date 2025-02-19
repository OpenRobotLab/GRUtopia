# Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

from typing import Any, Optional

import numpy as np
import torch
from modules.vlm.server_wrapper import (
    ServerMixin,
    host_model,
    send_request,
    str_to_image,
)
from PIL import Image

try:
    from lavis.models import load_model_and_preprocess
except ModuleNotFoundError:
    print('Could not import lavis. This is OK if you are only using the client.')


class BLIP2ITM:
    """BLIP 2 Image-Text Matching model."""

    def __init__(
        self,
        name: str = 'blip2_image_text_matching',
        model_type: str = 'pretrain',
        device: Optional[Any] = None,
    ) -> None:
        if device is None:
            device = torch.device('cuda') if torch.cuda.is_available() else 'cpu'

        (self.model, self.vis_processors, self.text_processors,) = load_model_and_preprocess(
            name=name,
            model_type=model_type,
            is_eval=True,
            device=device,
        )
        self.device = device

    def cosine(self, image: np.ndarray, txt: str) -> float:
        """
        Compute the cosine similarity between the image and the prompt.

        Args:
            image (numpy.ndarray): The input image as a numpy array.
            txt (str): The text to compare the image to.

        Returns:
            float: The cosine similarity between the image and the prompt.
        """
        pil_img = Image.fromarray(image)
        img = self.vis_processors['eval'](pil_img).unsqueeze(0).to(self.device)
        txt = self.text_processors['eval'](txt)
        with torch.inference_mode():
            cosine = self.model({'image': img, 'text_input': txt}, match_head='itc').item()

        return cosine


class BLIP2ITMClient:
    def __init__(self, port: int = 12182):
        self.url = f'http://localhost:{port}/blip2itm'

    def cosine(self, image: np.ndarray, txt: str) -> float:
        print(f'BLIP2ITMClient.cosine: {image.shape}, {txt}')
        response = send_request(self.url, image=image, txt=txt)
        return float(response['response'])


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=12182)
    args = parser.parse_args()

    print('Loading model...')

    class BLIP2ITMServer(ServerMixin, BLIP2ITM):
        def process_payload(self, payload: dict) -> dict:
            image = str_to_image(payload['image'])
            return {'response': self.cosine(image, payload['txt'])}

    blip = BLIP2ITMServer()
    print('Model loaded!')
    print(f'Hosting on port {args.port}...')
    host_model(blip, name='blip2itm', port=args.port)
