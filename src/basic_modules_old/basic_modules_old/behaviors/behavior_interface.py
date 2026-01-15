#!/usr/bin/env python3

from abc import ABC, abstractmethod
from typing import Dict, Any, Tuple

class BehaviorBase(ABC):
    @staticmethod
    @abstractmethod
    def get_description() -> Dict[str, Any]:
        """Return metadata: name, params, description
        
        Returns:
            Dict containing:
                name: String - behavior name
                type: int - behavior type identifier  
                description: String - human readable description
                params: List[Dict] - parameter definitions with name, type, required, default
        """
        pass

    @abstractmethod
    def setup_communication(self, node) -> None:
        """Attach publishers/subscribers to provided rclpy.Node"""
        pass

    @abstractmethod
    def set_params(self, params: Dict[str, Any]) -> None:
        """Configure behavior instance with given parameters"""
        pass

    @abstractmethod
    def execute_step(self) -> Tuple[bool, str, bool]:
        """Execute one controller tick
        
        Returns:
            Tuple[bool, str, bool]: (success, message, goal_reached)
        """
        pass

    @abstractmethod
    def reset(self) -> None:
        """Reset behavior state and clean up resources"""
        pass