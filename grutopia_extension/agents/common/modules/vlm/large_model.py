import base64
import os
from http import HTTPStatus

import dashscope
import tiktoken
from modules.vlm.prompt import TEMPLATE
from openai import AzureOpenAI, OpenAI
from PIL import Image


class Vlm_qwen:
    def __init__(
        self, verbose=False, try_times=20, agent_path='GRUtopia/grutopia_extension/agents/social_navigation_agent'
    ):
        self.agent_path = agent_path
        self.verbose = verbose
        self.try_times = try_times
        dashscope.api_key_file_path = os.path.join(agent_path, 'modules/vlm/api_key/aliyun_key.txt')

    def get_answer(self, template_type, **kwargs):
        image_bufs = kwargs.get('images', None)
        cnt = 0
        prompt = generate_prompt(template_type, **kwargs)
        content = [{'text': prompt}]
        if image_bufs is not None:
            for im_id, image_buf in enumerate(image_bufs):
                Image.open(image_buf).save(os.path.join(self.agent_path, f'images/vlm_image_{im_id}.png'))
                img_path = os.path.abspath(os.path.join(self.agent_path, f'images/vlm_image_{im_id}.png'))
                image_buf.close()
                content.append({'image': f'file://{img_path}'})
        while cnt < self.try_times:
            try:
                if image_bufs is not None:
                    messages = [{'role': 'user', 'content': content}]
                    response = dashscope.MultiModalConversation.call(
                        model='qwen-vl-plus', messages=messages, stream=False
                    )
                    if response.status_code == HTTPStatus.OK:
                        result = response.output.choices[0].message.content[0]['text']
                        break
                    else:
                        continue
                else:
                    messages = [
                        {
                            'role': 'system',
                            'content': 'You never explain your answer and always keep your answers concise. Remember answering in English.',
                        },
                        {'role': 'user', 'content': content},
                    ]
                    response = dashscope.Generation().call(
                        model='qwen-7b-chat', messages=messages, result_format='message'
                    )
                    if response.status_code == HTTPStatus.OK:
                        result = response.output.choices[0].message.content
                        break
                    else:
                        continue
            except Exception as e:
                print(e)
                cnt += 1
                result = None
        if self.verbose:
            print('prompt:', prompt)
            print('result:', result)
        return result


class Vlm_gpt4o:
    def __init__(
        self,
        azure=False,
        verbose=False,
        use_embedding=False,
        try_times=20,
        agent_path='GRUtopia/grutopia_extension/agents/social_navigation_agent',
    ):
        self.agent_path = agent_path
        if azure:
            with open(
                os.path.join(agent_path, 'modules/vlm/api_key/azure_api_key.txt'),
                'r',
                encoding='utf-8',
            ) as file:
                api_key = file.read().strip()
            if use_embedding:
                with open(
                    os.path.join(agent_path, 'modules/vlm/api_key/azure_api_key_e.txt'),
                    'r',
                    encoding='utf-8',
                ) as file:
                    api_key_e = file.read().strip()
            self.vlm = AzureOpenAI(
                api_key=api_key,
                api_version='2024-04-01-preview',
                azure_endpoint='https://gpt-4o-pjm.openai.azure.com/',
            )
            if use_embedding:
                self.embedding = AzureOpenAI(
                    api_key=api_key_e,
                    api_version='2024-02-15-preview',
                    azure_endpoint='https://text-embedding-3-large-pjm.openai.azure.com/',
                )
        else:
            with open(
                os.path.join(agent_path, 'modules/vlm/api_key/api_key.txt'),
                'r',
                encoding='utf-8',
            ) as file:
                api_key = file.read().strip()
            self.embedding = self.vlm = OpenAI(api_key=api_key)
        self.verbose = verbose
        self.deployment_name = 'gpt-4o'
        self.deployment_name_e = 'text-embedding-3-large'
        self.token_calculator = TokenCalculate('gpt-4o')
        self.try_times = try_times

    def get_embedding(self, text):
        text = text.replace('\n', ' ')
        self.token_calculator.accumulate_token(embedding=text)
        return self.embedding.embeddings.create(input=[text], model=self.deployment_name_e).data[0].embedding

    def get_answer(self, template_type, **kwargs):
        image_bufs = kwargs.get('images', None)
        cnt = 0
        prompt = generate_prompt(template_type, **kwargs)
        content = [{'type': 'text', 'text': prompt}]
        if image_bufs is not None:
            for im_id, image_buf in enumerate(image_bufs):
                img_encoded = base64.b64encode(image_buf.getvalue()).decode('utf-8')
                image_buf.close()
                item = {
                    'type': 'image_url',
                    'image_url': {
                        'url': f'data:image/png;base64,{img_encoded}',
                        'detail': 'high',
                    },
                    'index': im_id,
                }
                content.append(item)
                self.token_calculator.accumulate_token(image_num=1)

        while cnt < self.try_times:
            try:
                response = self.vlm.chat.completions.create(
                    model=self.deployment_name,
                    messages=[
                        {'role': 'user', 'content': content},
                    ],
                    max_tokens=2048,
                    top_p=1,
                    frequency_penalty=0,
                    presence_penalty=0,
                )
                result = response.choices[0].message.content
                self.token_calculator.accumulate_token(prompt=prompt, result=result)
                break
            except Exception as e:
                print(e)
                cnt += 1
                result = None
        if self.verbose:
            print('prompt:', prompt)
            print('result:', result)
        return result


class TokenCalculate:
    def __init__(self, model_type):
        self.token_calculate = tiktoken.encoding_for_model(model_type)
        self.input_tokens = 0
        self.output_tokens = 0
        self.embedding_tokens = 0
        self.images = 0

    def accumulate_token(self, prompt=None, result=None, embedding=None, image_num=None):
        if prompt:
            self.input_tokens += len(self.token_calculate.encode(prompt))
        if result:
            self.output_tokens += len(self.token_calculate.encode(result))
        if embedding:
            self.embedding_tokens += len(self.token_calculate.encode(embedding))
        if image_num:
            self.images += image_num

    def calculate_money(
        self,
        money_per_image,
        money_per_input_token,
        money_per_output_token,
        money_per_embedding_token,
    ):
        money = (
            money_per_image * self.images
            + money_per_input_token * self.input_tokens
            + money_per_output_token * self.output_tokens
            + money_per_embedding_token * self.embedding_tokens
        )
        print('USD: ', money)
        return money


def generate_prompt(template_type, **kwargs):
    """
    Generate a complete prompt based on the template type and provided content.

    Parameters:
        template_type (str): The type of template to use.
        **kwargs: The content to fill in the template.

    Returns:
        str: The complete prompt.
    """

    template = TEMPLATE.get(template_type, None)
    if template is None:
        raise ValueError(f"Template type '{template_type}' not found.")
    prompt = template.format(**kwargs)
    return prompt
