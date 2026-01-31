#!/usr/bin/env python3

import math
import random
from typing import Dict, Any, Tuple
from .conditions_interface import ConditionBase

class Condition(ConditionBase):

    def __init__(self) -> None:
        self._node = None
        self._start_time = 0.0
        self._fire_time = 0.0
        self._duration_s = 5.0
        self._steepness = 6.0

    @staticmethod
    def get_description() -> Dict[str, Any]:
        return {
            "name": "time_sigmoid",
            "type": 7,
            "description": "Fires deterministically at a time sampled from a sigmoid distribution centered on the given duration (seconds).",
            "params": [
                {"name": "t", "type": "float64", "required": False, "default": 5.0},  # midpoint (p=0.5 at t seconds)
                {"name": "k", "type": "float64", "required": False, "default": 6.0}   # steepness factor (higher = steeper)
            ]
        }

    def setup_communication(self, node) -> None:
        # No subscriptions needed
        self._node = node
        self._start_time = self._node.get_clock().now().nanoseconds / 1e9
        # Sample fire time from sigmoid distribution
        u = random.random()
        slope = self._steepness / self._duration_s
        self._fire_time = self._duration_s - (1 / slope) * math.log(1 / u - 1)

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
        current_time = self._node.get_clock().now().nanoseconds / 1e9
        elapsed = current_time - self._start_time
        fired = elapsed >= self._fire_time
        return fired, f"elapsed={elapsed:.2f}s, fire_at={self._fire_time:.2f}s"

    def reset(self) -> None:
        if self._node:
            self._start_time = self._node.get_clock().now().nanoseconds / 1e9
            # Resample fire time
            u = random.random()
            slope = self._steepness / self._duration_s
            self._fire_time = self._duration_s - (1 / slope) * math.log(1 / u - 1)
        else:
            self._start_time = 0.0
            self._fire_time = 0.0
