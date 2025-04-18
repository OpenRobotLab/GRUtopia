import json
import logging
import os
import time

import pymongo
import pytest

global prefix_id

prefix_id = str(int(time.time() * 1000))


@pytest.hookimpl(hookwrapper=True, tryfirst=True)
def pytest_runtest_makereport(item, call):
    mongodb_uri = os.environ.get('DB_URL')
    mongodb_name = 'qa_test'
    mongo_client = pymongo.MongoClient(mongodb_uri, authSource=mongodb_name)
    mongodb = mongo_client[mongodb_name]
    grutopia_table = mongodb['grutopia_test']
    out = yield
    res = out.get_result()
    case_name = item.name
    case_prority_list = ['P0', 'P1', 'P2', 'P3']
    case_prority = ''
    for i in range(len(item.own_markers)):
        if item.own_markers[i].name in case_prority_list:
            case_prority = item.own_markers[i].name
            break
    task_id = os.environ.get('JOB_ID') + '_' + prefix_id
    if res.when == 'setup':
        set_up_result = {}
        if res.outcome == 'passed':
            tmp_outcome = ''
        else:
            tmp_outcome = res.outcome
        set_up_result = {
            'task_id': task_id,
            'commit_id': os.environ.get('CI_COMMIT_SHA'),
            'case_name': case_name,
            'case_prority': case_prority,
            'nodeid': res.nodeid,
            'result': tmp_outcome,
            'stage': res.when,
            'detail': str(res.longrepr),
            'duration': res.duration,
            'begin_time': int(call.start * 1000),
            'end_time': int(call.stop * 1000),
        }
        result = set_up_result
        logging.info(f'set up stage result is:\n{result}')
        grutopia_table.insert_one(result)

    if res.when == 'call':
        metrics = {}
        if res.outcome == 'passed':
            if os.path.exists('./test_result.json'):
                with open('./test_result.json', 'r', encoding='utf-8') as f:
                    metrics = json.load(f)
        call_result = {
            'task_id': task_id,
            'commit_id': os.environ.get('CI_COMMIT_SHA'),
            'case_name': case_name,
            'case_prority': case_prority,
            'nodeid': res.nodeid,
            'result': res.outcome,
            'stage': res.when,
            'detail': str(res.longrepr),
            'duration': res.duration,
            'begin_time': int(call.start * 1000),
            'end_time': int(call.stop * 1000),
        }
        result = {**call_result, **metrics}
        logging.info(f'call stage result is:\n{result}')
        grutopia_table.update_one({'task_id': task_id, 'case_name': case_name, 'nodeid': res.nodeid}, {'$set': result})

    mongo_client.close()
