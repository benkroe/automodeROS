# Using Modules in Automode FSM Configurations

FSM configuration files (see [`fsm_configs.txt`](../scripe/fsm_configs.txt)) reference behavior and condition modules by their type IDs and allow passing parameters. The Automode controller dynamically loads the correct module based on the type mapping and injects parameters as specified.

## Example FSM Config Snippet

```text
--s0 0         # State 0 uses behavior type 0 (e.g., "exploration")
--c0x0 0       # Condition type 0 (e.g., "black_floor") for transition
```

## How It Works

1. The controller queries available modules and their descriptions at startup.
2. Each module provides a unique type ID and parameter list via its `get_description()` method.
3. The FSM config maps type IDs to module names using these descriptions.
4. Parameters specified in the config are passed to the module as defined in its description.

---

## Parameters

Parameters allow you to customize the behavior or condition modules for each state or transition in the FSM. In the FSM config, parameters are specified as additional flags after the type ID. These are mapped to the module's expected parameters by name or position.

- **Definition:** Each module should declare its expected parameters in the `get_description()` method, using the `"params"` field (a list of parameter names).
- **Mapping:** When the FSM is created, the controller matches parameters from the config to the module's parameter list. If you use `key=value` pairs in the config, they are mapped by name; otherwise, positional parameters are mapped in order.
- **Usage:** The parameters are passed to the module's `set_params(params: dict)` method as a dictionary.

### Example

Suppose your behavior module expects an `"rwm"` parameter:

```python
def get_description(self):
    return {
        "name": "exploration",
        "type": 0,
        "description": "Walks around random. Goes in one direction until obstacle is seen, turns around a random angle and goes forward again.",
        "params": [
            {"name": "rwm", "type": "int", "required": False, "default": 100}
        ]
    }
```

In the FSM config, you can specify:

```text
--s0 0 --rwm 50
```

This will result in `set_params({"rwm": 50})` being called on the module.

For conditions, you might have:

```python
def get_description(self):
    return {
        "name": "black_floor",
        "type": 0,
        "description": "Triggers when black floor is detected (or not detected) with probability (p)",
        "params": [
            {"name": "p", "type": "float64", "required": False, "default": 1}
        ]
    }
```

And in the config:

```text
--c0x0 0 --p 0.5
```

Which results in `set_params({"p": 0.5})` for the condition module.

---

## Extending the Package

To add new behaviors or conditions:

1. **Create a new Python file** in the appropriate subfolder:
    - Behaviors: `basic_modules/behaviors/`
    - Conditions: `basic_modules/conditions/`
2. **Implement the required class:**
    - For behaviors: a class named `Behavior`
    - For conditions: a class named `Condition`
3. **Implement the following methods:**
    - `get_description()`: Return a dict with a unique `type` (integer), a `name`, and a list of parameter names.
    - `set_params(params: dict)`: Accepts parameters for the module.
    - `reset()`: Resets internal state.
    - `setup_communication(node)`: (Optional) Setup ROS communication.
    - For behaviors: `execute_step()`
    - For conditions: `execute_reading()`
4. **Build the workspace.** Your new module will be available for use in Automode FSMs.

---

## Notes

- Type IDs must be unique across all modules of the same kind (behavior or condition).
- The controller automatically discovers and loads modules at runtime.
- Parameters in the FSM config are mapped to module parameters by name or position.

See [`fsm_configs.txt`](../scripe/fsm_configs.txt) for example configurations.
