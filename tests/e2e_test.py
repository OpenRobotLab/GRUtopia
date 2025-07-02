import json
import os
import subprocess
import sys

import pytest


def common_body(cmd_line):
    with subprocess.Popen(
        cmd_line,
        stdin=subprocess.PIPE,
        stderr=sys.stderr,
        close_fds=True,
        stdout=sys.stdout,
        universal_newlines=True,
        shell=True,
        bufsize=1,
    ) as cmd:
        cmd.communicate()
        assert cmd.returncode == 0, f'real exit code is {cmd.returncode}'


def update_jsonl_from_json(json_file_path, jsonl_file_path, update_item):
    with open(json_file_path, 'r', encoding='utf-8') as json_file:
        data = json.load(json_file)
        data = {**update_item, **data}
    if not isinstance(data, list):
        data = [data]
    with open(jsonl_file_path, 'a', encoding='utf-8') as jsonl_file:
        for item in data:
            json_line = json.dumps(item, ensure_ascii=False)
            jsonl_file.write(json_line + '\n')


def teardown_function(function):
    if os.path.exists('./test_result.json'):
        case_info = {}
        test_name = function.__name__
        case_info['case_info'] = test_name + '_' + os.environ.get('JOB_ID')
        update_jsonl_from_json('./test_result.json', '../total_result.jsonl', case_info)
    else:
        print('Warning! There is no test_result.json')


@pytest.mark.P0
def test_h1_locomotion():
    start_command = 'python ./tests/h1_locomotion.py'
    common_body(start_command)


@pytest.mark.P0
def test_h1_locomotion_multi_env():
    start_command = 'python ./tests/h1_locomotion_multi_env.py'
    common_body(start_command)


@pytest.mark.P0
def test_h1_locomotion_multi_env_endless():
    start_command = 'python ./tests/h1_locomotion_multi_env_endless.py'
    common_body(start_command)


@pytest.mark.P0
def test_h1_locomotion_3_env_2_episodes():
    start_command = 'python ./tests/h1_locomotion_3_env_2_episodes.py'
    common_body(start_command)


@pytest.mark.P0
def test_load_scene_without_robot():
    start_command = 'python ./tests/load_scene_without_robot.py'
    common_body(start_command)


@pytest.mark.P0
def test_load_scene_without_robot_multi_env():
    start_command = 'python ./tests/load_scene_without_robot_multi_env.py'
    common_body(start_command)


@pytest.mark.P0
def test_rep_camera_pointcloud():
    start_command = 'python ./tests/rep_camera_pointcloud.py'
    common_body(start_command)


@pytest.mark.P0
def test_franka_manipulation_multi_env_with_reset():
    start_command = 'python ./tests/franka_manipulation_multi_env_with_reset.py'
    common_body(start_command)


@pytest.mark.P0
def test_robots():
    start_command = 'set -e; ls ./tests/robots/*.py | grep -v test_ | while read f; do echo "run $f" && python $f; done'
    common_body(start_command)
