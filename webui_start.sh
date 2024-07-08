#!/bin/bash

set -eu

HOST=${WEBUI_HOST:-"127.0.0.1"}

echo "WebUI endpoint is set to http://${HOST}:8080"
sed -i "s/127.0.0.1:8211/${HOST}:8311/g" /usr/share/nginx/html/assets/index-*.js
sed -i "s/127.0.0.1/${HOST}/g" /usr/share/nginx/html/assets/index-*.js

# Start nginx, be and model(mock yet)
nginx
nginx -s reload
cd /isaac-sim/GRUtopia
nohup python ./webui/main.py &> webui.log &
