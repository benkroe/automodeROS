#!/usr/bin/env python3

from typing import Dict, Any, Tuple
from .conditions_interface import ConditionBase
import random

class Condition(ConditionBase):
    def __init__(self) -> None:
        self._node = None
        self._params: Dict[str, Any] = {}
        self._probability = 1.0  # Default probability (p parameter)

    @staticmethod
    def get_description() -> Dict[str, Any]:
        return {
            "name": "fixed_probability",
            "type": 5,
            "description": "Triggers randomly with fixed probability (p) each tick",
            "params": [
                {"name": "p", "type": "float64", "required": False, "default": 1.0}
            ]
        }

    def setup_communication(self, node) -> None:
        self._node = node  # No subscriptions needed

    def set_params(self, params: Dict[str, Any]) -> None:
        self._params.update(params)
        probability_param = params.get('p', 1.0)
        try:
            self._probability = float(probability_param)
        except (ValueError, TypeError):
            self._probability = 1.0

    def execute_reading(self) -> Tuple[bool, str]:
        if random.random() <= self._probability:
            return True, f"Triggered (p: {self._probability:.2f})"
        else:
            return False, f"Not triggered (p: {self._probability:.2f})"

    def reset(self) -> None:
        pass