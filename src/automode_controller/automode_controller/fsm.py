#!/usr/bin/env python3

from dataclasses import dataclass
from typing import Dict, List, Optional, Any
from enum import Enum

@dataclass
class FSMEdge:
   # Edge (Condition)
    condition_name: str
    condition_params: List[str]
    target_state: str

@dataclass
class FSMState:
   # State (Behavior)
    behavior_name: Optional[str] = None
    behavior_params: List[str] = None
    outgoing_edges: List[FSMEdge] = None
    
    # solves problem with same list in different instances (see python dataclass)
    def __post_init__(self):
        if self.behavior_params is None:
            self.behavior_params = []
        if self.outgoing_edges is None:
            self.outgoing_edges = []

class FSM:
    # FSM with all necessary funcitons
    
    def __init__(self, initial_state: str):
        self.states: Dict[str, FSMState] = {}
        self.current_state: str = initial_state
        self.initial_state: str = initial_state
    
    def add_state(self, state: FSMState):
        self.states[state.behavior_name] = state

    def add_edge(self, from_state: str, edge: FSMEdge):
        if from_state in self.states:
            self.states[from_state].outgoing_edges.append(edge)
    
    def get_current_state(self) -> FSMState:
        return self.states.get(self.current_state)
    
    def get_outgoing_edges(self, state_name: str = None) -> List[FSMEdge]:
        state_name = state_name or self.current_state
        return self.states.get(state_name, FSMState("")).outgoing_edges
    
    def transition_to(self, new_state: str) -> bool:
        if new_state in self.states:
            old_state = self.current_state
            self.current_state = new_state
            return True
        return False
    
    def reset(self):
        self.current_state = self.initial_state
    
    def get_state_names(self) -> List[str]:
        return list(self.states.keys())

def create_simple_fsm() -> FSM:
    # Simple example FSM for debugging and coding
    # -> wants to stop on black floor but more beeing alone
    fsm = FSM("exploration")
    
    
    exploration_state = FSMState(
        behavior_name="exploration",
        behavior_params=["30"]  
    )
    
    stop_state = FSMState(
        behavior_name="stop",  
    )
    
    fsm.add_state(exploration_state)
    fsm.add_state(stop_state)
    
    fsm.add_edge("exploration", FSMEdge(
        condition_name="black_floor",
        condition_params=[],
        target_state="stop",
    ))

    fsm.add_edge("stop", FSMEdge(
        condition_name="neighbour_count",
        condition_params=["1"],  
        target_state="exploration",
    ))
    
    return fsm