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
    name: str
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
        self.states[state.name] = state
    
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
    fsm = FSM("EXPLORATION")
    
    
    exploration_state = FSMState(
        name="EXPLORATION",
        behavior_name="explore",
        behavior_params=["30"]  
    )
    
    stop_state = FSMState(
        name="STOP",
        behavior_name="stop",  
    )
    
    fsm.add_state(exploration_state)
    fsm.add_state(stop_state)
    
    fsm.add_edge("EXPLORATION", FSMEdge(
        condition_name="black_floor",
        condition_params=[],
        target_state="STOP",
    ))
    
    fsm.add_edge("EXPLORATION", FSMEdge(
        condition_name="neighbour_count",
        condition_params=["1"],  
        target_state="EXPLORATION",
    ))
    
    return fsm


def create_fsm_from_config(fsm_config: str, behavior_descriptions: Any, condition_descriptions: Any) -> FSM:
    try:
        print(f"FSM DEBUG: Received behavior descriptions: {list(behavior_descriptions.keys())}")
        print(f"FSM DEBUG: Received condition descriptions: {list(condition_descriptions.keys())}")
        print(f"FSM DEBUG: Config string: {fsm_config}")

        # Step 1: Check how many states we have
        tokens = fsm_config.strip().split()
        print(f"FSM DEBUG: Tokens: {tokens}")
        
        nstates = _extract_nstates(tokens)
        print(f"FSM DEBUG: Creating FSM with {nstates} states")
        
        # Step 2: Separate the states
        state_configs = _separate_states(tokens, nstates)
        print(f"FSM DEBUG: Separated {len(state_configs)} state configurations")
        
        # Step 3: Clean parameter names
        cleaned_states = _clean_parameter_names(state_configs)
        print("FSM DEBUG: Cleaned parameter names")
        
        # Step 4: Validate against descriptions
        _validate_against_descriptions(cleaned_states, behavior_descriptions, condition_descriptions)
        print("FSM DEBUG: Validation passed")
        
        # Step 5: Create FSM with states and edges
        fsm = _create_fsm_from_cleaned_states(cleaned_states, behavior_descriptions, condition_descriptions)
        print("FSM DEBUG: FSM created successfully")
        
        return fsm
        
    except Exception as e:
        print(f"FSM ERROR: Failed to create FSM from config: {e}")
        import traceback
        print(f"FSM ERROR: Traceback: {traceback.format_exc()}")
        print("FSM DEBUG: Falling back to simple FSM")
        return create_simple_fsm()

def _extract_nstates(tokens: List[str]) -> int:
    """Step 1: Extract number of states from --nstates parameter."""
    try:
        nstates_index = tokens.index("--nstates")
        if nstates_index + 1 >= len(tokens):
            raise ValueError("--nstates parameter missing value")
        return int(tokens[nstates_index + 1])
    except ValueError as e:
        if "--nstates" in str(e):
            raise ValueError("--nstates parameter not found in config")
        else:
            raise ValueError("--nstates value must be a number")

def _separate_states(tokens: List[str], nstates: int) -> List[Dict[str, Any]]:
    """Step 2: Separate states and their conditions."""
    state_configs = []
    
    # Find state start positions
    state_positions = []
    for i, token in enumerate(tokens):
        if token.startswith("--s") and len(token) > 3 and token[3:].isdigit():
            state_num = int(token[3:])
            state_positions.append((state_num, i))
    
    # Sort by state number and validate sequence
    state_positions.sort()
    for i, (state_num, pos) in enumerate(state_positions):
        if state_num != i:
            raise ValueError(f"States must be sequential starting from 0. Missing state {i}")
    
    if len(state_positions) != nstates:
        raise ValueError(f"Expected {nstates} states, found {len(state_positions)}")
    
    # Extract each state's tokens
    for i, (state_num, start_pos) in enumerate(state_positions):
        end_pos = state_positions[i + 1][1] if i + 1 < len(state_positions) else len(tokens)
        state_tokens = tokens[start_pos:end_pos]
        
        # Parse state configuration
        state_config = _parse_state_tokens(state_tokens, state_num)
        state_configs.append(state_config)
    
    return state_configs

def _parse_state_tokens(tokens: List[str], state_num: int) -> Dict[str, Any]:
    """Parse tokens for a single state into structured data."""
    if len(tokens) < 2:
        raise ValueError(f"State {state_num} missing type")
    
    state_type = int(tokens[1])
    state_params = {}
    conditions = []
    
    i = 2
    # Parse state parameters (until we hit --n{state_num})
    while i < len(tokens) and not (tokens[i].startswith(f"--n{state_num}") and "x" not in tokens[i]):
        if i + 1 >= len(tokens):
            raise ValueError(f"State {state_num}: Parameter {tokens[i]} missing value")
        
        param_name = tokens[i]
        param_value = tokens[i + 1]
        state_params[param_name] = param_value
        i += 2
    
    # Parse conditions
    if i < len(tokens) and tokens[i] == f"--n{state_num}":
        if i + 1 >= len(tokens):
            raise ValueError(f"State {state_num}: --n{state_num} missing value")
        
        num_conditions = int(tokens[i + 1])
        i += 2
        
        # CHANGE: Parse each condition dynamically instead of fixed 6 tokens
        for cond_idx in range(num_conditions):
            condition = _parse_condition_tokens(tokens, i, state_num, cond_idx)
            conditions.append(condition)
            
            # CHANGE: Skip to next condition by finding next --n{state}x{cond+1} or end
            while i < len(tokens):
                if tokens[i].startswith(f"--n{state_num}x{cond_idx + 1}") or \
                   tokens[i].startswith("--s") or \
                   (cond_idx == num_conditions - 1):
                    break
                i += 1
    
    return {
        'state_num': state_num,
        'state_type': state_type,
        'params': state_params,
        'conditions': conditions
    }

def _parse_condition_tokens(tokens: List[str], start_idx: int, state_num: int, cond_idx: int) -> Dict[str, Any]:
    """Parse tokens for a single condition."""
    condition = {}
    i = start_idx
    
    # CHANGE: More flexible parsing to handle variable parameters like --w1x0
    while i < len(tokens):
        token = tokens[i]
        
        # Stop if we hit the next condition or next state
        if (token.startswith(f"--n{state_num}x") and token != f"--n{state_num}x{cond_idx}") or \
           token.startswith("--s") or \
           (token.startswith("--n") and "x" not in token):
            break
        
        # Parse this condition's tokens
        if token == f"--n{state_num}x{cond_idx}":
            condition['transition_type'] = int(tokens[i + 1])
        elif token == f"--c{state_num}x{cond_idx}":
            condition['condition_type'] = int(tokens[i + 1])
        elif token == f"--p{state_num}x{cond_idx}":
            condition['target_state_offset'] = int(tokens[i + 1])
        elif token == f"--w{state_num}x{cond_idx}":
            # CHANGE: Handle additional parameters like --w
            condition['w_param'] = int(tokens[i + 1])
        # Add more parameter types as needed
        
        i += 2
    
    return condition

def _clean_parameter_names(state_configs: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """Step 3: Clean parameter names by removing trailing digits."""
    def clean_param_name(param_token: str) -> str:
        if param_token.startswith("--"):
            # Remove leading -- and trailing digits
            clean_name = param_token[2:]
            while clean_name and clean_name[-1].isdigit():
                clean_name = clean_name[:-1]
            return clean_name
        else:
            raise ValueError(f"Parameter token '{param_token}' does not start with '--'")
    
    cleaned_states = []
    for state_config in state_configs:
        cleaned_params = {}
        for param_token, value in state_config['params'].items():
            clean_name = clean_param_name(param_token)
            cleaned_params[clean_name] = value
        
        cleaned_config = state_config.copy()
        cleaned_config['params'] = cleaned_params
        cleaned_states.append(cleaned_config)
    
    return cleaned_states

def _validate_against_descriptions(cleaned_states: List[Dict[str, Any]], 
                                 behavior_descriptions: Dict[str, Any], 
                                 condition_descriptions: Dict[str, Any]) -> None:
    """Step 4: Validate states and conditions against available descriptions."""
    
    # CHANGE: Create type-to-name mappings instead of using array indices
    behavior_type_map = {}
    condition_type_map = {}
    
    # Build behavior type mapping
    for name, desc in behavior_descriptions.items():
        behavior_type = desc.get('type', 0)  # Default to 0 if no type
        behavior_type_map[behavior_type] = name
    
    # Build condition type mapping  
    for name, desc in condition_descriptions.items():
        condition_type = desc.get('type', 0)  # Default to 0 if no type
        condition_type_map[condition_type] = name
    
    print(f"FSM DEBUG: Behavior type map: {behavior_type_map}")
    print(f"FSM DEBUG: Condition type map: {condition_type_map}")
    
    errors = []
    
    for state in cleaned_states:
        state_num = state['state_num']
        state_type = state['state_type']
        
        # CHANGE: Validate behavior exists by type instead of array index
        if state_type not in behavior_type_map:
            available_types = list(behavior_type_map.keys())
            errors.append(f"State {state_num}: Behavior type {state_type} not found. Available types: {available_types}")
        else:
            behavior_name = behavior_type_map[state_type]
            print(f"FSM DEBUG: State {state_num}: Using behavior '{behavior_name}' (type {state_type})")
        
        # CHANGE: Validate conditions by type instead of array index
        for cond_idx, condition in enumerate(state['conditions']):
            condition_type = condition['condition_type']
            
            if condition_type not in condition_type_map:
                available_types = list(condition_type_map.keys())
                errors.append(f"State {state_num}, Condition {cond_idx}: Condition type {condition_type} not found. Available types: {available_types}")
            else:
                condition_name = condition_type_map[condition_type]
                print(f"FSM DEBUG: State {state_num}, Condition {cond_idx}: Using condition '{condition_name}' (type {condition_type})")
    
    if errors:
        error_msg = "FSM Configuration Validation Errors:\n" + "\n".join(f"  ERROR: {error}" for error in errors)
        raise ValueError(error_msg)

def _create_fsm_from_cleaned_states(cleaned_states: List[Dict[str, Any]], 
                                   behavior_descriptions: Dict[str, Any], 
                                   condition_descriptions: Dict[str, Any]) -> FSM:
    """Step 5: Create FSM with states and edges."""
    
    # CHANGE: Create type-to-name mappings (same as validation)
    behavior_type_map = {}
    condition_type_map = {}
    
    for name, desc in behavior_descriptions.items():
        behavior_type = desc.get('type', 0)
        behavior_type_map[behavior_type] = name
    
    for name, desc in condition_descriptions.items():
        condition_type = desc.get('type', 0)
        condition_type_map[condition_type] = name
    
    # Create FSM with first state as initial
    fsm = FSM(f"STATE_{cleaned_states[0]['state_num']}")
    
    # Create all states first
    for state_config in cleaned_states:
        state_num = state_config['state_num']
        state_type = state_config['state_type']
        params = state_config['params']
        
        # CHANGE: Use type mapping instead of array index
        behavior_name = behavior_type_map[state_type]
        behavior_params = [params.get(key, "") for key in params.keys()]
        
        # CHANGE: Fix FSMState constructor to include name parameter
        state = FSMState(
            name=f"STATE_{state_num}",  # Now part of constructor
            behavior_name=behavior_name,
            behavior_params=behavior_params
        )
        
        fsm.add_state(state)
        print(f"FSM DEBUG: Created state STATE_{state_num} with behavior '{behavior_name}', params: {behavior_params}")
    
    # Create edges
    for state_config in cleaned_states:
        state_num = state_config['state_num']
        
        for condition in state_config['conditions']:
            condition_type = condition['condition_type']
            target_offset = condition['target_state_offset']
            
            condition_name = condition_type_map[condition_type]
            
            # FIX: Use target_offset directly as target state number
            target_state_num = target_offset
            
            edge = FSMEdge(
                condition_name=condition_name,
                condition_params=[],  # Add specific params if needed
                target_state=f"STATE_{target_state_num}"
            )
            
            fsm.add_edge(f"STATE_{state_num}", edge)
            print(f"FSM DEBUG: Added edge: STATE_{state_num} --[{condition_name}]--> STATE_{target_state_num}")
    
    return fsm



# let us create the create_fsm_from_config from scratch again. we need to do the following:
# 1.Check how much states we have. thats the value after --nstates
# 2.Seperate the states. They always start with --s{state_number} and as value the type of the state.
# Then in each state we need to seperate the conditions. The conditions beginn with --n{state_number} and the param which gives how many conditions (edges there are) In this ever individual condition(edge) starts with --n{state_number}x{condition number} and the param is the state they are directing to but -1 if the own state is lower than the goal state

# 3. If we have the individual states with their individual condition, we need to clean them. The params are then like --rwm0 but the states later can only have rwm as param thats true for the most params and also for the conditon names. 

# 4. in the last step check every state and condition against the behavior_descriptions and condition_descriptions if they are vailable. There we need good error logs

# 5. At last we need to create the states and edges.

# This is a example fsm i will explain the individual tokens:

# --fsm-config --nstates 2 --s0 0 --rwm0 50 --n0 2 --n0x0 0 --c0x0 0 --p0x0 0 --n0x1 0 --c0x1 0 --p0x1 0 --s1 1 --n1 1 --n1x0 0 --c1x0 3 --p1x0 1 --w1x0 0 

# --fsm-config -> indicates that the config starts
# --nstates 2 -> these two tokens indicate that we have two states
# -- s0 0 -> indicates the first state with type 0
# --rwm0 50 -> is the first param (in this case the only one) for the first state with the value 50
# --n0 2 -> indicates that the first state has two conditions
# --n0x0 0 --> first sate first transition transition type 0 (the transition type doent matter)
# --c0x0 0 --> indicates the condition from goes from first state to second state and has typ 0
# --p0x0 0 --> param for the first condition of the first state (goes from first to second state) and has value 0
# --n0x1 0 --> indicates the second condition for the first state also from 
# --c0x1 0 --> indicates condition goes from first state to second state and has type 0
# --p0x1 0 --> indicates the param for the condition p with the value 0
# --s1 1 --> indicates now starts the second state with typ 1
# --n1 1 --> indicates the first state has 1 condition
# --n1x0 0 --> start of first condition for the second state with condition type 0 (type doesnt matter)
# --c1x0 3 --> the condition for state 1  the first one has type 3
# -- p1x0 1 --> param (p) for the first condition of the second state with value 0
# --w1x0 0 -->param for the first conditoin of the second state (param.is w, value is 0)
