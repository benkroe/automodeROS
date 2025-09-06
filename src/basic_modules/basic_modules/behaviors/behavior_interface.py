#!/usr/bin/env python3

from abc import ABC, abstractmethod
from typing import Dict, Any, Tuple

class BehaviorBase(ABC):
    @staticmethod
    @abstractmethod
    def get_description() -> Dict[str, Any]:
        """Return metadata: name, params, description"""
        """It needs:
          name: String
          type: String --> is used later in the config
          description: String
          params: [param: {name: String, type: Any, required: Boolean, default: Any}]
          """

    @abstractmethod
    def setup_communication(self, node) -> None:
        """Attach pubs/subs to provided rclpy.Node"""

    @abstractmethod
    def set_params(self, params: Dict[str, Any]) -> None:
        """Configure behavior instance"""

    @abstractmethod
    def execute_step(self) -> Tuple[bool, str]:
        """Publish/act for one controller tick; return (success, message)"""

    def reset(self) -> None:
        """Optional: tear down publishers/subscriptions"""
        return