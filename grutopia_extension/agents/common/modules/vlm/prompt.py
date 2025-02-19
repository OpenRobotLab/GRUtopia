# flake8: noqa
TEMPLATE = {
    'get_goal': (
        'To better answer the question, the agent first should understand'
        + ' which object is the core in the question.'
        + '\nPlease give the output without any adjective according to the examples below:'
        + '\ninput: Can you locate the cabinet that has a single faucet hole without an installed faucet?'
        + '\noutput: cabinet'
        + "\ninput: What's the color of the blanket in the living room?"
        + '\noutput: blanket'
        + '\ninput: {question}'
        + '\noutput:'
    ),
    'get_pick_and_place_all': (
        'To better understand the instruction, the agent should decompose the instruction into 3 parts'
        + '\nPICK_GOAL: The target to pick.'
        + '\nPICK_GOAL_DESCRIPTION: Descriptions about the pick goal.'
        + '\nPLACE_GOAL: The target to place.'
        + '\nPLACE_GOAL_DESCRIPTION: Descriptions about the place goal.'
        + '\nRELATIONSHIP: The target spacial relationship between the pick goal and place goal.'
        + '\nPlease give the output according to the examples below:'
        + '\ninput: Pick the bowl that has another bowl inside it, which is near a cabinet, a chair, and below another cabinet. Place it near a plate that can hold the object and near another bowl.'
        + '\noutput:'
        + '\nPICK_GOAL: bowl'
        + '\nPICK_GOAL_DESCRIPTION: has another bowl inside it, which is near a cabinet, a chair, and below another cabinet'
        + '\nPLACE_GOAL: plate'
        + '\nPLACE_GOAL_DESCRIPTION: can hold the object and near another bowl'
        + '\nRELATIONSHIP: near'
        + '\ninput: {question}'
        + '\noutput:'
        + '\ninput: Pick the pillow in the bedroom, which is near the curtain, and place it near the cup and the bowl.'
        + '\noutput:'
        + '\nPICK_GOAL: pillow'
        + '\nPICK_GOAL_DESCRIPTION: in the bedroom, which is near the curtain'
        + '\nPLACE_GOAL: cup'
        + '\nPLACE_GOAL_DESCRIPTION: and the bowl'
        + '\nRELATIONSHIP: near'
    ),
    'extract_info': (
        'You are an expert in extracting information from a dialogue to complete a info template.'
        + '\n## Info Template You Can Use'
        + '\nThe goal is in [ROOM].'
        + '\nThe goal is not in [ROOM].'
        + '\nThe goal [SPATIAL RELATION].'
        + '\nThe goal [ATTRIBUTE].'
        + '\n\n## Introduction to the task'
        + '\nYou will be given a dialogue relative to the object. Your task is to extract information about the goal object and identify which type this information belongs to(ROOM, SPATIAL RELATION or ATTRIBUTE). '
        + 'And complete the corresponding template above. Your outputs should be strictly follow this form, put one output in one line:'
        + '\n[INFO TYPE], [COMPLETED TEMPLATE]'
        + '\n\n## An Example'
        + '\ninput_question: Could you please give me more infos about the goal box? input_answer: The goal box is a white rectangular box with gold-colored metal accents located in the bathroom. It has a bottle on it.'
        + '\noutput:'
        + '\nATTRIBUTE, The goal is a white rectangular box with gold-colored metal accents.'
        + '\nROOM, The goal is in the bathroom.'
        + '\nSPATIAL RELATION, The goal has no bottles on it.'
        + '\n\n## Your question:'
        + '\ninput question:{question} input answer:{answer}'
        + '\noutput:'
    ),
    'choose_candidate': (
        "You are an expert in choosing candidates based on given goal information and candidates' images."
        + '\n\n## Introduction to the task'
        + '\nYou are the brain of an embodied agent whose task is to find a specific {goal} in an unseen environment. You will be given some information about the goal {goal} and images of some candidates for the {goal}. '
        + 'What you need to do is to choose which candidate is most possible to be the goal.'
        + '\n\n## Description about the input'
        + '\nHere are some information about the goal {goal}:'
        + '\n{goal_info}'
        + '\nHere are the ordered images of the candidates, you can refer to a candidate by its image index:'
        + '\n{image_prompt}'
        + '\n\n## Instruction about the output'
        + "\n1. You need to carefully compare the candidates' images with the given goal object information. Choose the most possible candidate's image index as output."
        + "\n2. Please only output one most possible candidate's image index."
        + '\n3. Note that never explain your decision, just output the chosen image index, do not fabricate an image index.'
        + '\noutput:'
    ),
    'ask_question': (
        'You are an expert in asking question to sift given candidates.'
        + '\n\n## Introduction to the task'
        + '\nYou are the brain of an embodied agent whose task is to find a specific {goal} in an unseen environment. You will be given some information about the goal {goal} and images of some candidates for the {goal}. '
        + 'What you need to do is to ask a question to best determine whether the given candidates are the goal object.'
        + '\n\n## Description about the input'
        + "\nHere are some information about the goal {goal}, they are all true, you don't need to question them:"
        + '\n{goal_info}'
        + '\nHere are the ordered images of the candidates, you can refer to a candidate by its image index:'
        + '\n{image_prompt}'
        + '\n\n## Instruction about the output'
        + "\n1. You need to carefully compare the candidates' images with the given goal object information. Ask the most effective question that can help you to sift the candidates."
        + '\n2. Note that never explain your decision, just output the question.'
        + "\n3. You need ask question according to the image but note that the respondent can not see current image, so please don't ask question like: "
        + "\'Is there a cabinet near the bed in the image?\'"
        + '\n4. You need to best use the information contained in the image!'
        + '\noutput:'
    ),
    'check_candidates': (
        '# Task instruction'
        + '\nYou will be given an accurate description of the target object {goal} and a set of four images showing the same candidate object_{label} from different viewpoints, '
        + 'where the candidate object_{label} in each image is labeled with a number {label} in text. '
        + '\nPlease determine whether the candidate object_{label} in the image matches the description of the target object {goal} and '
        + 'provide an output of either -1, 0, or 1 based on the following criteria:'
        + '\n-1: The candidate object_{label} annotated with {label} in the image is impossible to be the target object. '
        + '(e.g., the target object is a table, but the object labeled with number in the image is a bed).'
        + '\n0: The candidate object_{label} annotated with {label} in the image may be the target object, but you think the description is insufficient or unclear to confirm with confidence.'
        + '\n1: The candidate object_{label} annotated with {label} in the image is very likely the target object because it satisfies all of the description criteria sufficiently '
        + 'and you think the description is enough to confirm with confidence.'
        + '\n\n# Input:'
        + 'A description of the target object (it can be incomplete, but it must be accurate):'
        + '\nThe goal object is {goal}.'
        + '\n{goal_info}'
        + 'One image includes four sub_images showing the same object from different viewpoints with the object_{label} labeled {label} in text.'
        + '\nSee image attached.'
        + '\n\n## Instruction about the output'
        + '\n1. You need to carefully compare the object_{label} in the image with the given goal object information. '
        + '\n2. Note that never explain your decision, just output -1 or 0 or 1.'
    ),
}
