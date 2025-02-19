# Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

from typing import Any, Set

import numpy as np


class StateAction:
    def __init__(self, position: np.ndarray, action: Any, other: Any = None):
        self.position = position
        self.action = action
        self.other = other

    def __hash__(self) -> int:
        string_repr = f'{self.position}_{self.action}_{self.other}'
        return hash(string_repr)


class AcyclicEnforcer:
    history: Set[StateAction] = set()

    def check_cyclic(self, position: np.ndarray, action: Any, other: Any = None) -> bool:
        state_action = StateAction(position, action, other)
        cyclic = state_action in self.history
        return cyclic

    def add_state_action(self, position: np.ndarray, action: Any, other: Any = None) -> None:
        state_action = StateAction(position, action, other)
        self.history.add(state_action)
