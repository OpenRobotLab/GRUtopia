import argparse

import uvicorn
from fastapi import FastAPI

from webui.src.routes import router


def parse_arg():
    parser = argparse.ArgumentParser(description='Welcome to use GRUtopia WebUI. Try to use it in a right way please.')
    parser.add_argument('--host', type=str, default='0.0.0.0', help='The host address to listen on. Default: 0.0.0.0')
    parser.add_argument('--port', type=int, default=9000, help='The port to listen on. Default: 8000.')
    return parser.parse_args()


app = FastAPI()
app.include_router(router)

if __name__ == '__main__':
    args = parse_arg()
    uvicorn.run(app, host=args.host, port=args.port)
