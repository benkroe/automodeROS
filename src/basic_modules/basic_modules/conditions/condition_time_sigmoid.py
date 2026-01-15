#!/usr/bin/env python3

import time
import random
from typing import Dict, Any, Tuple
from .conditions_interface import ConditionBase

class Condition(ConditionBase):

    def __init__(self) -> None:
        self._start_time = time.time()
        self._duration_s = 5.0
        self._steepness = 6.0

    @staticmethod
    def get_description() -> Dict[str, Any]:
        return {
            "name": "time_sigmoid",
            "type": 7,
            "description": "Triggers with sigmoid probability centered on the given duration (seconds).",
            "params": [
                {"name": "t", "type": "float64", "required": False, "default": 5.0},  # midpoint (p=0.5 at t seconds)
                {"name": "k", "type": "float64", "required": False, "default": 6.0}   # steepness factor (higher = steeper)
            ]
        }

    def setup_communication(self, node) -> None:
        # No subscriptions needed
        self._start_time = time.time()

    def set_params(self, params: Dict[str, Any]) -> None:
        try:
            self._duration_s = float(params.get("t", 5.0))
            if self._duration_s <= 0:
                self._duration_s = 1.0
        except (TypeError, ValueError):
            self._duration_s = 5.0
        try:
            self._steepness = float(params.get("k", 6.0))
            if self._steepness <= 0:
                self._steepness = 1.0
        except (TypeError, ValueError):
            self._steepness = 6.0

    def execute_reading(self) -> Tuple[bool, str]:
        elapsed = time.time() - self._start_time
        # Sigmoid centered at duration: p=0.5 when elapsed==duration
        slope = self._steepness / self._duration_s
        p = 1.0 / (1.0 + pow(2.718281828459045, -slope * (elapsed - self._duration_s)))
        fired = random.random() <= p
        return fired, f"elapsed={elapsed:.2f}s, p={p:.3f}"

    def reset(self) -> None:
        self._start_time = time.time()
