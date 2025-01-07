filter_num_prompt = """
You are an expert in extracting information from a dialogue to complete a info template.

## Introduction to the task
You will be given a dialogue relative to the goal object.
Your task is to extract information about the goal object and identify which type this information belongs to(room, relation or appearance).

## Instruction about the output
1. When you think you know from the dialogue that the goal is in a room, choose template:
room: (yes, room)
2. When you think you know from the dialogue that the goal is not in a room, choose template:
room: (no, room)
3. When you think you know from the dialogue that the goal has some spatial relation with others, choose template:
relation: (yes, relation_type, category)
4. When you think you know from the dialogue that the goal hasn't got some spatial relation with others, choose template:
relation: (no, relation_type, category)
5. When you think you know from the dialogue that the goal has some attributes neither as room nor spatial relation, choose template:
appearance: (attribute)
6. When you think according to the dialogue, the goal object has already be found, choose template:
done
7. When you think you can extract nothing from the dialogue, choose template:
-1
8. Note that the relation_type in relation format could only be chosen from relation_type as follows:
near, in, on, above, below, contain/hold, under.
9. Your answer must be strictly composed from the six given format above, and do not need any explanations.
10. You can choose more than one template to complete, but you must use \\n to separate them.

## Some Examples
goal category: faucet.
input_question: Is the goal object located in a bathroom or kitchen?
input_answer: the goal object is located in the bathroom.
output:
room: (yes, bathroom)

goal category: cabinet.
input_question: Could you please describe the location of the cabinet in relation to other objects or landmarks in the room?
input_answer: There are no bottles on the cabinet.
output:
relation: (no, on, bottles)

goal category: box.
input_question: Could you tell me more about the goal object?
input_answer: the goal object is a white rectangular box in the living room.
output:
room: (yes, living room)
appearance: (the goal object is a white rectangular box)

goal category: bed.
input_question: Is what I am facing to the goal?
input_answer: No. Its not the bed Im looking for.
output: -1

## Current input
goal category: {category}
input_question: {question}
input_answer: {answer}
output:
"""
