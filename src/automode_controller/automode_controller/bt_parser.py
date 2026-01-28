"""Parser for --bt-config strings into a tree specification.

Format notes (examples accepted):
- Tokens are space-separated, flags start with `--` and have a following value.
- Node type: `--n<ID> <type>` where <ID> is a digit sequence like `0`, `00`, `10`.
- Child counts: `--nchild<ID> <count>` or `--nchildroot <count>` for the top-level root children.
- Root type: `--nroot <type>` (optional; if absent a default Selector is assumed).
- Top-level children: produced as `0..(nchildroot-1)` unless explicit node ids differ.
- Leaf details: per-node params use short-prefix keys where the suffix is the node id digits,
  e.g. `--a01 2` (action id), `--c00 1` (condition id), `--p01 0` (param index/value), etc.

The parser returns a dict with a `root` entry and `nodes` mapping node-id -> spec.
Each node spec: { 'type': int, 'children': [ids], 'params': {key: value, ...} }
"""

from typing import Dict, Any, List
import re
import logging
logging.basicConfig(level=logging.INFO)


def _tokenize(config_str: str):
    parts = config_str.strip().split()
    # remove leading --bt-config token if present
    if parts and parts[0].startswith('--bt-config'):
        parts = parts[1:]
    tokens = []
    i = 0
    while i < len(parts):
        key = parts[i]
        if not key.startswith('--'):
            i += 1
            continue
        key = key[2:]
        val = None
        if i + 1 < len(parts) and not parts[i+1].startswith('--'):
            val = parts[i+1]
            i += 2
        else:
            val = ''
            i += 1
        tokens.append((key, val))
    return tokens


def parse_bt_config(config_str: str) -> Dict[str, Any]:
    """Parse a bt-config string into a structured spec.

    Returns:
      {
        'root': {'type': int, 'children': [ids]},
        'nodes': { id: {'type': int, 'children': [...], 'params': {...}} }
      }
    """
    tokens = _tokenize(config_str)

    node_types: Dict[str, int] = {}
    node_children_counts: Dict[str, int] = {}
    node_params: Dict[str, Dict[str, Any]] = {}
    root_type = None
    root_children = None

    # simple key regexes
    re_n = re.compile(r'^n(\d+)$')
    re_nchild = re.compile(r'^nchild(\d+)$')

    for k, v in tokens:
        # root special keys
        if k == 'nroot':
            try:
                root_type = int(v)
            except Exception:
                root_type = None
            continue
        if k == 'nchildroot':
            try:
                root_children = int(v)
            except Exception:
                root_children = None
            continue

        # node type (n<ID>)
        m = re_n.match(k)
        if m:
            nid = m.group(1)
            try:
                node_types[nid] = int(v)
            except Exception:
                node_types[nid] = None
            continue

        # nchild<ID>
        m = re_nchild.match(k)
        if m:
            nid = m.group(1)
            try:
                node_children_counts[nid] = int(v)
            except Exception:
                node_children_counts[nid] = 0
            continue

        # parameter-like keys ending with node id digits
        m = re.match(r'^([a-zA-Z]+)(\d+)$', k)
        if m:
            pfx = m.group(1)
            nid = m.group(2)
            node_params.setdefault(nid, {})[pfx] = _coerce_value(v)
            continue

        # unknown key: store at top-level params
        node_params.setdefault('_root', {})[k] = _coerce_value(v)

    # Build node specs
    nodes: Dict[str, Dict[str, Any]] = {}

    # Initialize nodes from node_types entries
    for nid, ntype in node_types.items():
        nodes[nid] = {'type': ntype, 'children': [], 'params': node_params.get(nid, {})}

    # If any node param exists without explicit type, create node entry
    for nid, params in node_params.items():
        if nid == '_root':
            continue
        if nid not in nodes:
            nodes[nid] = {'type': None, 'children': [], 'params': params}

    # Attach children based on nchild counts: children are nid + index
    for nid, cnt in node_children_counts.items():
        parent = nodes.setdefault(nid, {'type': None, 'children': [], 'params': node_params.get(nid, {})})
        child_ids = []
        for i in range(cnt):
            cid = f"{nid}{i}"
            # create placeholder child if not present
            if cid not in nodes:
                nodes[cid] = {'type': None, 'children': [], 'params': node_params.get(cid, {})}
            child_ids.append(cid)
        parent['children'] = child_ids

    # For nodes that were specified with n<ID> but not given children count,
    # infer children if there exist nodes that start with that id and are longer by one digit
    for nid in list(nodes.keys()):
        if nodes[nid]['children']:
            continue
        # find immediate children with prefix nid and length == len(nid)+1
        prefix = nid
        immediate = [k for k in nodes.keys() if k.startswith(prefix) and len(k) == len(prefix) + 1]
        if immediate:
            nodes[nid]['children'] = sorted(immediate)

    # Validation: if root_children was provided, ensure sequential single-digit node ids exist
    if root_children is not None:
        expected = [str(i) for i in range(root_children)]
        missing = [e for e in expected if e not in nodes]
        if missing:
            raise ValueError(f"BT config: nchildroot={root_children} but missing node ids: {missing}")

    # Clean parameter token names to remove trailing digits where applicable (FSM-style)
    def _clean_param_name(token: str) -> str:
        # token is like 'rwm01' or 'a0' already split; remove trailing digits
        s = token
        while s and s[-1].isdigit():
            s = s[:-1]
        return s

    # Apply cleaning to node params: keys are currently prefixes (letters), so this is a no-op
    # but keep the step for compatibility with FSM mechanics
    for nid, node in nodes.items():
        cleaned = {}
        for k, v in node.get('params', {}).items():
            clean_k = _clean_param_name(k)
            cleaned[clean_k] = v
        node['params'] = cleaned

    # Build root
    if root_type is None:
        root_type = 0  # default Selector

    if root_children is not None:
        root_children_ids = [str(i) for i in range(root_children)]
        # ensure nodes exist
        for cid in root_children_ids:
            nodes.setdefault(cid, {'type': None, 'children': [], 'params': node_params.get(cid, {})})
    else:
        # infer top-level nodes as single-digit ids present
        root_children_ids = sorted([k for k in nodes.keys() if len(k) == 1])

    root = {'type': root_type, 'children': root_children_ids}

    return {'root': root, 'nodes': nodes}


def load_categories_from_descriptions(behavior_descriptions: Dict[str, Any], condition_descriptions: Dict[str, Any]):
    """Build node and edge id->name maps from behavior/condition descriptions.

    Mirrors the FSM approach where each description contains a numeric `type` field.
    Returns (node_map, edge_map) where keys are ints and values are names.
    """
    node_map = {}
    edge_map = {}
    try:
        for name, desc in (behavior_descriptions or {}).items():
            t = desc.get('type')
            if t is not None:
                try:
                    node_map[int(t)] = name
                except Exception:
                    continue
    except Exception:
        pass

    try:
        for name, desc in (condition_descriptions or {}).items():
            t = desc.get('type')
            if t is not None:
                try:
                    edge_map[int(t)] = name
                except Exception:
                    continue
    except Exception:
        pass

    return node_map, edge_map


def _coerce_value(v: str):
    if v is None:
        return None
    if isinstance(v, (int, float)):
        return v
    vs = str(v)
    if vs.lower() in ('true', 'false'):
        return vs.lower() == 'true'
    try:
        iv = int(vs)
        return iv
    except Exception:
        pass
    try:
        fv = float(vs)
        return fv
    except Exception:
        pass
    return vs


def pretty_print_spec(spec: Dict[str, Any]) -> str:
    """Utility to pretty-print the parsed spec for debugging. Returns a string."""
    lines = []
    lines.append(f"Root: type={spec['root']['type']}, children={spec['root']['children']}")
    for nid, node in sorted(spec['nodes'].items()):
        lines.append(f"Node {nid}: type={node['type']}, children={node['children']}, params={node['params']}")
    return "\n".join(lines)


def validate_spec_against_categories(spec: Dict[str, Any], node_map: Dict[int, str], edge_map: Dict[int, str]):
    """Validate parsed BT spec against discovered behavior/condition categories.

    Raises ValueError on mismatch. Basic checks:
    - root children exist
    - action nodes (type 5) have either numeric 'a' that exists in node_map or a 'name'
    - condition nodes (type 6) have either numeric 'c' that exists in edge_map or a 'name'
    """
    nodes = spec.get('nodes', {})
    root = spec.get('root', {})

    # root children exist
    for cid in root.get('children', []):
        if cid not in nodes:
            raise ValueError(f"BT spec: root child '{cid}' not defined in nodes")

    for nid, node in nodes.items():
        ntype = node.get('type')
        params = node.get('params', {}) or {}

        if ntype == 5:  # Action / Behavior
            a = params.get('a')
            if a is None and 'name' not in params:
                raise ValueError(f"Action node '{nid}' missing 'a' (id) or 'name' parameter")
            if isinstance(a, int) and a not in node_map and node_map:
                raise ValueError(f"Action node '{nid}' references unknown behavior id {a}")

        if ntype == 6:  # Condition
            c = params.get('c')
            if c is None and 'name' not in params:
                raise ValueError(f"Condition node '{nid}' missing 'c' (id) or 'name' parameter")
            if isinstance(c, int) and c not in edge_map and edge_map:
                raise ValueError(f"Condition node '{nid}' references unknown condition id {c}")

    return True
