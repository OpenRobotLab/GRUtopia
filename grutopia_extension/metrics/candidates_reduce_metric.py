import difflib
import json
import pickle
import re

import numpy as np
import torch
from torch.nn.functional import cosine_similarity

np.random.seed(2024)
from collections import defaultdict
from typing import Union

from grutopia.core.config.metric import MetricUserConfig
from grutopia.core.runtime.task_runtime import TaskRuntime
from grutopia.core.task.metric import BaseMetric
from grutopia.core.util import log

from .prompt import filter_num_prompt

try:
    from openai import AzureOpenAI, OpenAI
except ModuleNotFoundError as e:
    log.error('ModuleNotFoundError: No module named \'openai\'. Please install it first.')
    raise e
try:
    from transformers import CLIPModel, CLIPProcessor
except ModuleNotFoundError as e:
    log.error('ModuleNotFoundError: No module named \'transformers\'. Please install it first.')
    raise e


@BaseMetric.register('ECRMetric')
class ECRMetric(BaseMetric):
    """
    Calculate the average reduced candidates per dialogue in this episode
    """

    def __init__(self, config: MetricUserConfig, task_runtime: TaskRuntime):
        super().__init__(config, task_runtime)
        self.target = task_runtime.extra['target']
        self.question = task_runtime.extra['question']
        self.object_dict_path = task_runtime.extra['object_dict_path']
        self.model_mapping_path = task_runtime.extra['model_mapping_path']
        self.captions_path = task_runtime.extra['captions_path']
        self.captions_embeddings_path = task_runtime.extra['captions_embeddings_path']

        if task_runtime.extra['use_azure']:
            with open(task_runtime.extra['azure_api_key_path'], 'r', encoding='utf-8') as file:
                api_key = file.read().strip()

            with open(task_runtime.extra['azure_api_key_e_path'], 'r', encoding='utf-8') as file:
                api_key_e = file.read().strip()

            self.llm = AzureOpenAI(
                api_key=api_key, api_version='2024-04-01-preview', azure_endpoint='https://gpt-4o-pjm.openai.azure.com/'
            )
            self.embedding = AzureOpenAI(
                api_key=api_key_e,
                api_version='2024-02-15-preview',
                azure_endpoint='https://text-embedding-3-large-pjm.openai.azure.com/',
            )
        else:
            with open(task_runtime.extra['api_key_path'], 'r', encoding='utf-8') as file:
                api_key = file.read().strip()
            self.embedding = self.llm = OpenAI(api_key=api_key)

        self.reset()

    def reset(self):
        self.candidates_reduced = []
        with open(self.object_dict_path, 'r') as file:
            spatial_relations = json.load(file)
        with open(self.model_mapping_path, 'r') as file:
            model_mapping = json.load(file)
        with open(self.captions_path, 'r') as file:
            object_caption = json.load(file)
        with open(self.captions_embeddings_path, 'rb') as file:
            captions_embeddings = pickle.load(file)

        self.dialogues = []
        self.question = None
        self.dialogue_graph = Dialogue_Graph(
            self.llm, self.embedding, object_caption, spatial_relations, captions_embeddings, model_mapping
        )

        all_candidates = [(obj_id, relation) for obj_id, relation in self.dialogue_graph.spatial_relations.items()]
        all_candidates = self.dialogue_graph.filter_candidates(
            {'cate': self.target.split('/')[0].lower()}, all_candidates
        )
        all_candidates = (
            self.filter_num(
                self.target.split('/')[0].lower(), {'question': self.question, 'answer': 'Yes.'}, all_candidates
            )
            if self.question
            else all_candidates
        )
        self.all_candidates = set([i[0] for i in all_candidates])
        self.seen_candidates = set()
        self.seen_candidates_with_time = {}

    def update(self, task_obs: dict):
        """
        This function is called at each world step.
        """
        robot_obs = task_obs[self.task_runtime.robots[0].name]
        landmarks = robot_obs['camera']['landmarks']
        if landmarks:
            seen_landmarks = [landmark.split(',')[1].lower() for landmark in landmarks if len(landmark.split(',')) == 2]
            new_seen_candidates = [can for can in list(set(self.all_candidates)) if can.lower() in seen_landmarks]
            self.seen_candidates.update(new_seen_candidates)

        if task_obs[self.task_runtime.robots[0].name].get('question') is not None:
            self.question = task_obs[self.task_runtime.robots[0].name].get('question')
        if task_obs[self.task_runtime.robots[0].name].get('answer') is not None:
            self.dialogues.append(
                {'question': self.question, 'answer': task_obs[self.task_runtime.robots[0].name].get('answer')}
            )
            chat_id = len(self.seen_candidates_with_time)
            self.seen_candidates_with_time[chat_id] = self.seen_candidates

    def calc(self, task_info: dict):
        """
        This function is called to calculate the metrics when the episode is terminated.
        """
        exclude_candidates = set()
        res_candidates = []
        for dialogue_idx, dialogue in enumerate(self.dialogues):
            name_set = self.seen_candidates_with_time[dialogue_idx] - exclude_candidates
            original_candidates = [(obj_id, self.dialogue_graph.spatial_relations[obj_id]) for obj_id in name_set]
            res_candidates = self.filter_num(self.target.split('/')[0].lower(), dialogue, original_candidates)
            self.candidates_reduced.append(
                {'ori_num': len(original_candidates), 'reduced_num': len(original_candidates) - len(res_candidates)}
            )
            exclude_candidates = exclude_candidates | (
                set([i[0] for i in original_candidates]) - set([i[0] for i in res_candidates])
            )
        candidates_reduced = np.mean(
            [float(i['reduced_num']) / float(i['ori_num']) for i in self.candidates_reduced if i['ori_num'] != 0]
        )
        candidates_reduced = float(candidates_reduced) if len(self.candidates_reduced) > 0 else -1
        log.info('ECRMetric calc() called.')
        ecr_metric = {
            'candidates_reduced': self.candidates_reduced,
            'candidates_reduced_num': candidates_reduced,
            'left_candidates': res_candidates,
        }
        return {k: v.tolist() if isinstance(v, np.float32) else v for k, v in ecr_metric.items()}

    def filter_num(self, category, npc_dialogue, candidates):
        try:
            filter_num_prompt.format(
                category=category, question=npc_dialogue['question'], answer=npc_dialogue['answer']
            )
            response = self.llm.chat.completions.create(
                model='gpt-4o',  # Make sure to use the correct model name
                messages=[{'role': 'user', 'content': filter_num_prompt}],
                max_tokens=100,
            )
            result = response.choices[0].message.content if response.choices else None
            print(result)
            if result is None or '-1' in result:
                return candidates
            if 'done' in result.lower():
                candidate_names = [i[0].lower() for i in candidates]
                if self.target.lower() in candidate_names:
                    return [(self.target, self.dialogue_graph.spatial_relations[self.target])]
                else:
                    return []
            results = [i.strip() for i in result.split('\n')]
            result_dicts = []
            for answer in results:
                pattern = r'(\w+)\s*:\s*\((.*?)\)'
                matches = re.findall(pattern, answer)
                for key, value in matches:
                    result_dicts.append({'key': key.strip(), 'value': value.strip()})
            npc_answer = {'room': [], 'relation': [], 'appearance': []}
            for relation in result_dicts:
                if relation['key'].lower() == 'room':
                    try:
                        value = [i.strip() for i in relation['value'].split(',')]
                        if value[0].lower() == 'yes':
                            npc_answer['room'].append((True, value[1]))
                        elif value[0].lower() == 'no':
                            npc_answer['room'].append((False, value[1]))
                    except Exception as e:
                        print(e)
                elif relation['key'].lower() == 'relation':
                    try:
                        value = [i.strip() for i in relation['value'].split(',')]
                        if value[0].lower() == 'yes':
                            npc_answer['relation'].append((True, value[1], value[2]))
                        elif value[0].lower() == 'no':
                            npc_answer['relation'].append((False, value[1], value[2]))
                    except Exception as e:
                        print(e)
                elif relation['key'].lower() in 'appearance':
                    try:
                        npc_answer['appearance'].append(relation['value'])
                    except Exception as e:
                        print(e)
            res_candidates = self.dialogue_graph.filter_candidates(npc_answer, candidates)
            return res_candidates
        except Exception as e:
            print(e)
            return candidates


def calc_similarity(e1: np.ndarray, e2: np.ndarray):
    return (e1 * e2).sum() / (np.linalg.norm(e1) * np.linalg.norm(e2))


class Dialogue_Graph:
    def __init__(
        self,
        llm: Union[AzureOpenAI, OpenAI],
        embedding: Union[AzureOpenAI, OpenAI],
        attribute_set: dict,
        spatial_relations: dict,
        attr_embedding,
        model_mapping: dict,
    ) -> None:
        self.llm = llm
        self.embedding = embedding
        self.attribute_set = attribute_set
        self.spatial_relations = spatial_relations
        self.attr_embedding = attr_embedding
        self.id2cate = {}
        self._preprocess(model_mapping)
        self.reset()
        self.candidates = {}
        self.model = CLIPModel.from_pretrained('openai/clip-vit-base-patch32')
        self.processor = CLIPProcessor.from_pretrained('openai/clip-vit-base-patch32')

    def _preprocess(self, model_mapping):
        temp_attribute_set = {}
        for obj_id in self.spatial_relations:
            key = model_mapping.get(obj_id)
            if key not in self.attribute_set:
                continue
            temp_attribute_set[obj_id] = self.attribute_set[key]

        self.attribute_set = temp_attribute_set

        embedding = {}
        for oid, attrs in self.attr_embedding.items():
            for cp, eb in attrs:
                embedding[cp] = eb

        self.attr_embedding = embedding

        for k, v in self.spatial_relations.items():
            self.id2cate[k] = v['category']

    def reset(self):
        self.candidates = {}
        self.nodes = {}
        self.visited = set()
        self.sampled = set()

    def add_node(self, node_id):
        assert node_id not in self.nodes, f'The node {node_id} has already been added.'
        if node_id not in self.attribute_set:
            attributes = []
        else:
            attributes = self.attribute_set[node_id]
        relations = self.spatial_relations[node_id]['nearby_objects']
        node = {}
        node['node_id'] = node_id
        node['attributes'] = attributes
        node['relations'] = relations
        node['remain'] = len(attributes) + len(relations)

        self.nodes[node_id] = node

    def _node_probablity(self, nodes):
        p = np.array([self.nodes[n]['remain'] for n in nodes])
        p = p / p.sum()
        return p

    def sample_node(self):
        nodes = list(self.nodes.keys())
        p = self._node_probablity(nodes)
        node = np.random.choice(nodes, p=p)

        return node

    def sample_attribute(self, node_id):
        assert node_id in self.nodes, f'The node {node_id} is not in the node set.'
        node = self.nodes[node_id]
        attributes = node['attributes']
        attr_score = [_[0] for _ in attributes]
        attr_content = [_[1] for _ in attributes]
        # for _ in attr_content:
        #     attr = _
        #     key = node_id + "/" + attr
        #     if not key in self.visited:
        #         self.visited.add(key)
        #         break
        if len(attr_content) == 0:
            return None
        attr_score = -np.log(np.array(attr_score))
        p = attr_score / attr_score.sum()
        cnt = 0
        while cnt < 50:
            attr = np.random.choice(attr_content, p=p)
            key = node_id + '/' + attr
            if key not in self.visited:
                self.visited.add(key)
                break
            cnt += 1
        # assert cnt < 50, "Exceed the maximum sample time. No attribute is sampled."
        if cnt == 50:
            return None
        self.nodes[node_id]['remain'] -= 1
        return attr

    def sample_relation(self, node_id):
        assert node_id in self.nodes, f'The node {node_id} is not in the node set.'
        node = self.nodes[node_id]
        relations = node['relations']
        relations_by_type = defaultdict(list)
        for target_object, [relation, dist] in relations.items():
            target_object_cate = target_object.split('/')[0]
            if target_object_cate == 'other':
                continue

            if relation == 'near' and dist > 0.5:
                continue

            relations_by_type[relation].append((target_object, dist))

        relation_types = list(relations_by_type.keys())
        if len(relation_types) == 0:
            return None, None

        mean_dist = []
        for type in relation_types:
            dists = [_[1] for _ in relations_by_type[type]]
            mean_dist.append(np.mean(dists))

        mean_dist = np.array(mean_dist) + 1
        p = 1.0 / mean_dist
        p = p / p.sum()

        cnt = 0
        while cnt < 50:
            cnt += 1
            sampled_type = np.random.choice(relation_types, p=p)
            relations = relations_by_type[sampled_type]
            # keys = []
            flag = False
            relations_filtered = []
            object_to_add = []
            for relation in relations:
                target_object, _ = relation
                target_object_cate = target_object.split('/')[0]
                key = node_id + '/' + target_object
                if key not in self.visited:
                    flag = True
                if target_object not in self.nodes:
                    if target_object_cate != 'other' and target_object_cate != 'others':
                        relations_filtered.append(relation)
                        object_to_add.append(target_object)

                    self.nodes[node_id]['remain'] -= 1
                    # self.add_node(target_object)

                # keys.append(key)
                self.visited.add(key)

            if flag and len(relations_filtered) != 0:
                for obj_id in object_to_add:
                    self.add_node(obj_id)
                break

            # assert cnt < 50, "Exceed the maximum sample time. No relation is sampled."
        if cnt == 50:
            return None, None

        length = len(relations_filtered)
        rel_idx = np.random.randint(length)
        sampled_relation = relations_filtered[rel_idx]
        return [sampled_relation], sampled_type

    def sample_event(self):
        res = {}
        node_id = self.sample_node()
        res['node_id'] = node_id
        if np.random.rand() > 0.8:  # sample an attribute
            attr = self.sample_attribute(node_id)
            if attr is None:
                relations, relation_type = self.sample_relation(node_id)
                res['event_type'] = 'relation'
                res['event'] = (relation_type, relations)
            else:
                res['event_type'] = 'attribute'
                res['event'] = attr
        else:
            relations, relation_type = self.sample_relation(node_id)
            if relations is None:
                attr = self.sample_attribute(node_id)
                res['event_type'] = 'attribute'
                res['event'] = attr
            else:
                res['event_type'] = 'relation'
                res['event'] = (relation_type, relations)

        return res

    def extract_info(self, event):
        node_id = event['node_id']
        cate = node_id.split('/')[0]
        if event['event_type'] == 'attribute':
            attr = event['event']
            info = f"""The target object is a {cate}. {attr}"""
        else:
            relation_type, relations = event['event']
            info = """["""
            cnt = defaultdict(int)
            for obj, dist in relations:
                obj_cate = obj.split('/')[0]
                cnt[obj_cate] += 1
            for key, num in cnt.items():
                info += f""" \"{num} {key} is/are {relation_type} the {cate}.\","""
            info += ']'
        return info, event['event_type']

    def extract_info_v2(self, info):
        if 'cate' not in info:
            info['cate'] = 'object'
        cate = info['cate']
        res_str = ''
        if 'room' in info:
            room = info['room']
            splits = room.split('_')
            if splits[-1] in [str(_) for _ in range(10)]:
                splits = splits[:-1]
            room = ' '.join(splits)
            res_str += f"""This object is in {room}."""
        if 'relation' in info:
            cnt = defaultdict(int)
            true_or_false = {}
            for _, relation_type, obj_cate in info['relation']:
                cnt[obj_cate] += 1
                true_or_false[obj_cate] = _
            for key, num in cnt.items():
                tof = true_or_false[key]

                if key == 'nothing':
                    if tof:
                        res_str += f"""nothing is {relation_type} the {cate}."""
                    else:
                        res_str += f"""something is {relation_type} the {cate}."""
                else:
                    if tof:
                        res_str += f""" There is/are {num} {key} {relation_type} the {cate}."""
                    else:
                        res_str += f""" There is/are no {key} {relation_type} the {cate}."""

        if 'appearance' in info:
            for attr in info['appearance']:
                res_str += attr

        return res_str

    def find_difference(self, candidates, current_id=None):
        if current_id is None or current_id not in candidates:
            candidates = list(candidates)
            id_idx = np.random.randint(len(candidates))
            current_id = candidates[id_idx]

        candidates = [(obj_id, self.spatial_relations[obj_id]) for obj_id in candidates]

        cates = set()
        rooms = set()
        relations = defaultdict(list)

        current_relation = {}

        for (obj_id, relation) in candidates:
            cate = relation['category']
            room = relation['room']
            if cate != 'others':
                cates.add(cate)
            rooms.add(room)
            relation_sets = {'near': defaultdict(int)}

            for target, (rel_type, dist) in relation['nearby_objects'].items():
                target_cate = target.split('/')[0]
                if target_cate != 'other' and target_cate != 'others':
                    if rel_type not in relation_sets:
                        relation_sets[rel_type] = defaultdict(int)

                    relation_sets[rel_type][target_cate] += 1
                    if rel_type != 'near':
                        if dist < 1:
                            relation_sets['near'][target_cate] += 1

            for key, item in relation_sets.items():
                relations[key].append(item)
                if obj_id == current_id:
                    current_relation[key] = item
            # print(relations)
        # category
        if len(cates) > 1:
            length = [len(_) for _ in cates]
            length = min(length)
            all = set()
            for _ in cates:
                all.add(_[:length].lower())

            if len(all) > 1:
                return 'category', cates, current_id  # {current_id} is in {room}

        # rooms
        temp_room = set([_[:4] for _ in rooms])
        if len(temp_room) > 1:
            return 'room', rooms, current_id  # {current_id} is in {room}

        # print(relations)
        # relations
        for rel_type, target_list in relations.items():
            # print(rel_type, target_list)
            # if len(target_list) < len(candidates):
            n = len(candidates) - len(target_list)
            for i in range(n):
                target_list.append(defaultdict(int))

            if rel_type not in current_relation:
                return 'relation', (rel_type, 'nothing'), current_id

            target_dict_current = current_relation[rel_type]

            for i, target_dict_a in enumerate(target_list):
                for key in target_dict_current.keys():
                    if key not in target_dict_a:
                        return 'relation', (rel_type, key), current_id

        return 'appearance', None, current_id

    def get_more_info(self, diff, node_id):
        item_rel = self.spatial_relations[node_id]
        item_attr = self.attribute_set[node_id]
        diff_type, diff_content, _ = diff
        if diff_type == 'category':
            return {'cate': item_rel['category']}

        if diff_type == 'room':
            return {'room': item_rel['room']}

        if diff_type == 'relation':
            rel_type_target, target_cate = diff_content

            relations = item_rel['nearby_objects']
            flag = False
            rel_type_set = set()
            for obj_id, (rel_type, dist) in relations.items():
                cate = obj_id.split('/')[0]
                rel_type_set.add(rel_type)
                if rel_type_target == rel_type and target_cate == cate:
                    flag = True
                    break
                if rel_type_target == 'near' and dist < 1 and target_cate == cate:
                    flag = True
                    break

            if target_cate == 'nothing':
                if rel_type_target in rel_type_set:
                    return {'relation': [(False, rel_type_target, 'nothing')]}
                else:
                    return {'relation': [(True, rel_type_target, 'nothing')]}

            return {'relation': [(flag, rel_type_target, target_cate)]}
        attr = None
        len_item_attr = len(item_attr)
        attr_idx = np.random.randint(len_item_attr)
        _, attr = item_attr[attr_idx]
        assert attr is not None
        # while len(self.sampled) < len_item_attr:
        #     attr_idx = np.random.randint(len_item_attr)
        #     _, attr = item_attr[attr_idx]
        #     if not attr in self.sampled:
        #         self.sampled.add(attr)
        #         break

        return {'appearance': [attr]}

    def filter_candidates(self, infos, candidates=None):
        '''
        infos:
        {
            "cate": str, optional
            "room": [(flag, room), ...]optional
            "relation": [(has_or_not, relation_type, cate), ...]
            "appearance": [attr1, attr2]
        }
        candidates: optional, [obj_id, ]
        '''
        res_candidates = set()

        if candidates is None:
            candidates = [(obj_id, relation) for obj_id, relation in self.spatial_relations.items()]

        for obj_id, relation in candidates:
            if obj_id in self.attribute_set:
                attrs = self.attribute_set[obj_id]
            else:
                attrs = []

            if 'cate' in infos:
                cate_info = self.processor(text=[infos['cate']], images=None, return_tensors='pt', padding=True)
                cate_item = self.processor(text=[relation['category']], images=None, return_tensors='pt', padding=True)

                with torch.no_grad():
                    cate_info = self.model.get_text_features(**cate_info)
                    cate_item = self.model.get_text_features(**cate_item)

                similarity = cosine_similarity(cate_info, cate_item)
                if not (similarity > 0.8):
                    # if cate_item != 'others':
                    continue

            if 'room' in infos:
                # room_info = infos['room']
                # room_item = relation['room']
                # min_len = min(len(room_info), len(room_item))
                # if not room_info[:min_len].lower() == room_item[:min_len].lower():
                #     continue
                room_info = infos['room']
                room_item = relation['room']
                may_be_candidate = True
                for r_info in room_info:
                    # min_len = min(len(r_info[1]), len(room_item))
                    if not r_info[0]:
                        # if r_info[1][:min_len].lower() == room_item[:min_len].lower():
                        #     may_be_candidate = False
                        #     break
                        if (
                            difflib.get_close_matches(r_info[1].lower(), [room_item.lower()], n=1, cutoff=0.9)
                            or r_info[1].lower() in room_item.lower()
                        ):
                            may_be_candidate = False
                            break
                    else:
                        # if r_info[1][:min_len].lower() != room_item[:min_len].lower():
                        #     may_be_candidate = False
                        #     break
                        if (
                            difflib.get_close_matches(r_info[1].lower(), [room_item.lower()], n=1, cutoff=0.7)
                            or r_info[1].lower() not in room_item.lower()
                        ):
                            may_be_candidate = False
                            break
                if not may_be_candidate:
                    continue

            if 'relation' in infos:
                relation_info = infos['relation']
                relation_item = relation['nearby_objects']
                flag = True
                for rel_info in relation_info:
                    if not flag:
                        break

                    has_or_not, rel_a, cate_a = rel_info
                    if has_or_not:
                        flag_has = False
                        rel_b_set = set()
                        for key, (rel_b, dist) in relation_item.items():
                            rel_b_set.add(rel_b)
                            cate_b = key.split('/')[0]
                            if rel_a == rel_b and (
                                difflib.get_close_matches(cate_a.lower(), [cate_b], n=1, cutoff=0.7)
                                or (cate_a == cate_b)
                            ):  # noqa: W503
                                flag_has = True
                                break

                            if (
                                rel_a == 'near'
                                and dist < 1
                                and (
                                    difflib.get_close_matches(cate_a.lower(), [cate_b], n=1, cutoff=0.7)
                                    or (cate_a == cate_b)
                                )
                            ):
                                flag_has = True
                                break

                        if cate_a == 'nothing':  # if has nothing rel_a the object
                            if rel_a in rel_b_set:  # there should not be any rel_a in this candidate
                                flag = False
                        else:
                            if not flag_has:
                                flag = False
                                # break
                    else:
                        rel_b_set = set()
                        for key, (rel_b, dist) in relation_item.items():
                            rel_b_set.add(rel_b)
                            cate_b = key.split('/')[0]
                            if rel_a == rel_b and (
                                difflib.get_close_matches(cate_a.lower(), [cate_b], n=1, cutoff=0.9)
                                or (cate_a == cate_b)
                            ):  # noqa: W503
                                flag = False
                                break

                            if (
                                rel_a == 'near'
                                and dist < 1
                                and (
                                    difflib.get_close_matches(cate_a.lower(), [cate_b], n=1, cutoff=0.9)
                                    or (cate_a == cate_b)
                                )
                            ):
                                flag = False
                                break

                        if cate_a == 'nothing':  # does not have nothing, means has something
                            if rel_a not in rel_b_set:
                                flag = False

                if not flag:
                    continue

            # if 'appearance' in infos:
            #     app_info = infos['appearance']
            #     app_item = attrs
            #     flag = True
            #     for app_a in app_info:
            #         eb_a = self._get_embedding(app_a)
            #         flag_has = False
            #         for score, app_b in app_item:
            #             eb_b = self.attr_embedding[app_b]
            #             if calc_similarity(eb_a, eb_b) > 0.9:
            #                 flag_has = True
            #                 break
            #         if not flag_has:
            #             flag = False
            #             break

            #     if not flag:
            #         continue

            if 'appearance' in infos and infos['appearance']:
                app_info = ' '.join(infos['appearance'])
                app_item = ' '.join([s[1] for s in attrs])
                if not self._may_be_same(app_info, app_item):
                    continue

            res_candidates.add(obj_id)
        res_candidates = [(candidate, self.spatial_relations[candidate]) for candidate in list(res_candidates)]
        return res_candidates

    def close_to_eachother(self, candidates):
        candidates = list(candidates)
        flag = False
        for i, obj_id in enumerate(candidates):
            relation = self.spatial_relations[obj_id]['nearby_objects']
            close_to = True
            for obj_id_b in candidates[i + 1 :]:
                if obj_id_b not in relation:
                    close_to = False
                    break

                _, dist = relation[obj_id_b]
                if dist > 0.1:
                    close_to = False
                    break

            if close_to:
                flag = True
                break

        return flag

    def get_difference(self, current, target):
        current.append(target)
        candidates = set(current)
        diff = self.find_difference(candidates, current_id=target)
        difference_info = self.get_more_info(diff=diff, node_id=target)
        if category := difference_info.get('cate'):
            return f"The object I'm looking for is a {category}.", difference_info
        elif room := difference_info.get('room'):
            if room.startswith('bedroom') or room.startswith('toilet'):
                room = room.split('_')[0]
            elif room in ['corridor', 'kitchen']:
                pass
            elif room in ['living_room', 'dining_room']:
                room = ' '.join(room.split('_'))
            return f'The {self.id2cate[target]} is located in the {room}.', difference_info
        elif relation := difference_info.get('relation'):
            flag, rel_type_target, target_cate = relation[0]
            if flag:
                if target_cate != 'nothing':
                    return f'There is a {target_cate} {rel_type_target} the {self.id2cate[target]}.', difference_info
                else:
                    return f'There is nothing {rel_type_target} the {self.id2cate[target]}.', difference_info
            else:
                if target_cate != 'nothing':
                    return f'There is no {target_cate} {rel_type_target} the {self.id2cate[target]}.', difference_info
                else:
                    return f'There is something {rel_type_target} the {self.id2cate[target]}.', difference_info
        elif appearance := difference_info.get('appearance'):
            return f'{appearance[0]}', difference_info
        else:
            raise ValueError('No difference found.')

    def _get_embedding(self, text):
        text = text.replace('\n', ' ')
        return self.embedding.embeddings.create(input=[text], model='text-embedding-3-large').data[0].embedding

    def _may_be_same(self, app_info: str, app_item: str):
        language_prompt = (
            'This is the description for the goal object:'
            + f'\n{app_info}'
            + 'This is the description for current object:'
            + f'\n{app_item}'
            + '\nDo you think they could be the same object?'
            + '\nYour answer should only be a single yes or no, do not need any explanations.'
        )
        response = self.llm.chat.completions.create(
            model='gpt-4o',  # Make sure to use the correct model name
            messages=[{'role': 'user', 'content': language_prompt}],
            max_tokens=100,
        )
        result = response.choices[0].message.content if response.choices else None

        try:
            if 'no' in result.lower():
                return False
        except Exception as e:
            print(e)

        return True
