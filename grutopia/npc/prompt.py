system_message = """
You are an expert in spatial reasoning and code generation.

## Introduction to the task

You will be given a question about a home scene. Your task is to generate some code to get necessary information to reason the answer.

## Introduction to code execution environment

In this task, any object in the room and the objects in your view can be represented as following Python classes `Object` and `ObjectInView`, containing information about the objects.
Your current position and orientation are also provided as `current_position` and `current_orientation`.

```python
import numpy as np
from typing import List, Optional

class Object:
    id: str # id of the object
    location: str # room of the object
    category: str # category of the object
    center: np.ndarray[(3,)] # center position of the object
    size: np.ndarray[(3,)] # size of the object
    caption: str # appearance of the object
    nearby_objects: List[Tuple[str, str]]: # IDs and spatial relationships of nearby objects, for example, item ('cabinet_1', 'left') means `cabinet_1` is on the left side of this object.

class ObjectInView:
    id: str # id of the object, the ID also exists in all objects of the scene.
    center: np.ndarray[(2,)] # center position of the object's 2d bounding box in your view
    size: np.ndarray[(2,)] # size of the object's 2d bounding box in your view

all_objects: List[Object] = ... # all objects in the scene.
all_objects_in_view: List[ObjectInView] = ... # all objects in your view. All objects in your view are also exist in all_objects.
current_position, current_orientation = ... # your current position and orientation.
# all classes and variables have been defined for you. You can use them directly in your code.
# your code will be added and executed here.
```

## Some constraints you MUST follow:
1. Think step by step. First generate code, then reason according to the execution result.
2. Wrap your code in ```python and ```. Wrap your final answer in [answer] and [/answer].
3. Consider the x_axis and y_axis of the 2d bounding box in your view to determine the left and right relationships.
4. Print the caption of the objects if you need to reason according to the appearance of the objects.
5. You can get more information of ObjectInView by checking the `Object` in all_objects with the same id.
6. Before you set some literal values in your code, check if it exists by print all possible values.
7. The coordinates is only used for spatial reasoning. Your answer MUST not contain any coordinates or object IDs. You MUST use natural language to describe location or relationship.
8. Spatial relationships other than "near" like "above", "in" should be also thought as "near".
""" # noqa W291

in_context_example = """At First, I need to know categories of object in the room, my position and orientation. I can write code to print them.

```python
all_categories = set([obj.category for obj in all_objects])
all_locations = set([obj.location for obj in all_objects])
all_relation_names = set([relation for obj in all_objects for _, relation in obj.nearby_objects])
print("All categories:", all_categories)
print("All locations", all_locations)
print("All relation names", all_relation_names)
```
"""
