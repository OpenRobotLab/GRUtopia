# Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

from typing import Any, Optional

import numpy as np
import torch
from PIL import Image

from .server_wrapper import ServerMixin, host_model, send_request, str_to_image

try:
    from lavis.models import load_model_and_preprocess
except ModuleNotFoundError:
    print('Could not import lavis. This is OK if you are only using the client.')


class BLIP2:
    def __init__(
        self,
        name: str = 'blip2_t5',
        model_type: str = 'pretrain_flant5xxl',
        device: Optional[Any] = None,
    ) -> None:
        if device is None:
            device = torch.device('cuda') if torch.cuda.is_available() else 'cpu'

        self.model, self.vis_processors, _ = load_model_and_preprocess(
            name=name,
            model_type=model_type,
            is_eval=True,
            device=device,
        )
        self.device = device

    def ask(self, image: np.ndarray, prompt: Optional[str] = None) -> str:
        """Generates a caption for the given image.

        Args:
            image (numpy.ndarray): The input image as a numpy array.
            prompt (str, optional): An optional prompt to provide context and guide
                the caption generation. Can be used to ask questions about the image.

        Returns:
            dict: The generated caption.

        """
        pil_img = Image.fromarray(image)
        with torch.inference_mode():
            processed_image = self.vis_processors['eval'](pil_img).unsqueeze(0).to(self.device)
            if prompt is None or prompt == '':
                out = self.model.generate({'image': processed_image})[0]
            else:
                out = self.model.generate({'image': processed_image, 'prompt': prompt})[0]

        return out


class BLIP2Client:
    def __init__(self, port: int = 12185):
        self.url = f'http://localhost:{port}/blip2'

    def ask(self, image: np.ndarray, prompt: Optional[str] = None) -> str:
        if prompt is None:
            prompt = ''
        response = send_request(self.url, image=image, prompt=prompt)

        return response['response']


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=8070)
    args = parser.parse_args()

    print('Loading model...')

    class BLIP2Server(ServerMixin, BLIP2):
        def process_payload(self, payload: dict) -> dict:
            image = str_to_image(payload['image'])
            return {'response': self.ask(image, payload.get('prompt'))}

    # blip = BLIP2Server(name="blip2_opt", model_type="pretrain_opt2.7b")
    blip = BLIP2Server(name='blip2_t5', model_type='pretrain_flant5xl')
    print('Model loaded!')
    print(f'Hosting on port {args.port}...')
    host_model(blip, name='blip2', port=args.port)
