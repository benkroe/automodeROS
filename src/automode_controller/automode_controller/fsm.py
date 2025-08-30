#!/usr/bin/env python3

from dataclasses import dataclass
from typing import Dict, List, Optional, Any
from enum import Enum
import logging

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




def create_fsm_from_config(fsm_config: str, behavior_descriptions: Any, condition_descriptions: Any) -> FSM:
    fsm = FSM("INITIAL")
    # Get n states
    _tokens = fsm_config.strip().split()
    if _tokens[0] != "--fsm-config":
        raise ValueError("FSM config must start with '--fsm-config'")
    if _tokens[1] != "--nstates":
        raise ValueError("FSM config must contain '--nstates'")
    _states_count = int(_tokens[2])
    logging.info(f"Parsing FSM config with {_states_count} states")

    _remaining_tokens = _tokens[3:]

    ########################
    # Split tokens by states
    state_configs = _split_config_into_config_states(_remaining_tokens, _states_count)

    #######################
    # Split state_configs in state and conditions
    for state_config in state_configs:
        _state, condition_tokens = _split_into_state_and_conditions(state_config)
        conditions = _split_condition_tokens(condition_tokens)
        
def _split_config_into_config_states(_remaining_tokens, _states_count):
    _state_configs = []
    current_state_tokens = []
    expected_state_number = 0

    for token in _remaining_tokens:
        # start of new state?
        if token.startswith("--s") and len(token) > 3:
            if token[3:].isdigit():
                state_number = int(token[3:])

                # check if state number is correct
                if state_number == expected_state_number:
                    _state_configs.append(current_state_tokens)
                    current_state_tokens = [token]
                    expected_state_number += 1
                else:
                    raise ValueError(f"Expected state number {expected_state_number}, got {state_number}")
            # if we have new valid state append old tokens to states (only if state_number is not 0)
                if current_state_tokens:
                    _state_configs.append(current_state_tokens)
                
                # start new state
                current_state_tokens = [token]
                expected_state_number += 1

        # add token to current state if no new state
        else:
            current_state_tokens.append(token)

    # Add the last state
    if current_state_tokens:
        _state_configs.append(current_state_tokens)
    
    # Validate if --nstates fits to state count
    if len(_state_configs) != _states_count:
        raise ValueError(f"Expected {_states_count} states, got {len(_state_configs)}")
        
    return _state_configs

def _split_into_state_and_conditions(state_config):
    state_params = []
    
    #### Split int state and conditons(one string)
    # Find where conditions start (--n{number})
    condition_start_index = None
    for i, token in enumerate(state_config):
        if token.startswith("--n") and len(token) > 3 and token[3:].isdigit():
            condition_start_index = i
            break
    
    if condition_start_index is None:
        # No conditions found, everything is state parameters
        state_params = state_config
        conditions = []
    else:
        # Split at the condition start
        state_params = state_config[:condition_start_index]
        condition_tokens = state_config[condition_start_index:]
    
    #### Split the conditions into seperated conditions
    conditions = _split_condition_tokens(condition_tokens)
    
    return state_params, conditions

def _split_condition_tokens(condition_tokens):
    # Split the condition tokens into individual conditions.
    conditions = []
    current_condition = []
    
    for token in condition_tokens:
        # Check if this is a new condition marker (--n{number}x{number})
        if token.startswith("--n") and "x" in token:
            # Save previous condition if exists
            if current_condition:
                conditions.append(current_condition)
            
            # Start new condition
            current_condition = [token]
        else:
            # Add token to current condition
            current_condition.append(token)
    
    # Add the last condition
    if current_condition:
        conditions.append(current_condition)
    
    return conditions

def _get_clean_para_name(_param_token: str) -> str:
    if _param_token.startswith("--"):
        # remove leading -- and trailing digits
        clean_name = _param_token[2:]
        while clean_name and clean_name[-1].isdigit():
            clean_name = clean_name[:-1]
        return clean_name
    else:
        raise ValueError(f"Parameter token '{_param_token}' does not start with '--'")



    return fsm