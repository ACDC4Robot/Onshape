"""URDF structure and physics checks without CAD or numerical dependencies."""
import math
from .serialization import NUMERIC_ATTRIBUTES, decimal_number

SUPPORTED_JOINTS = {"fixed", "revolute", "continuous", "prismatic"}
INERTIA_KEYS = ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")


def numbers(text, count, context):
    if text is None:
        raise ValueError(f"Missing {context}")
    tokens = text.split()
    if count is not None and len(tokens) != count:
        raise ValueError(f"{context} requires {count} numeric values")
    if not tokens:
        raise ValueError(f"Empty {context}")
    values = []
    for token in tokens:
        decimal_number(token)  # bounds expansion before writing fixed decimals
        value = float(token)
        if not math.isfinite(value):
            raise ValueError(f"{context} must be finite")
        values.append(value)
    return values


def scalar(node, attribute):
    return numbers(node.get(attribute), 1, f"{node.tag}.{attribute}")[0]


def determinant(matrix):
    a, b, c = matrix[0]
    d, e, f = matrix[1]
    g, h, i = matrix[2]
    return a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)


def physical_inertia(values):
    scale = max(abs(value) for value in values)
    if scale == 0:
        return False
    xx, yy, zz, xy, xz, yz = [value / scale for value in values]
    inertia = [[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]]
    if xx <= 0 or xx * yy - xy * xy <= 0 or determinant(inertia) <= 0:
        return False
    # A physical tensor also satisfies the principal-moment triangle
    # inequalities. Half-trace*I - inertia must be positive semidefinite.
    half_trace = (xx + yy + zz) / 2
    covariance = [[(half_trace if row == col else 0) - inertia[row][col]
                   for col in range(3)] for row in range(3)]
    tolerance = 1e-10
    if any(covariance[index][index] < -tolerance for index in range(3)):
        return False
    for first, second in ((0, 1), (0, 2), (1, 2)):
        minor = covariance[first][first] * covariance[second][second] - covariance[first][second] ** 2
        if minor < -tolerance:
            return False
    return determinant(covariance) >= -tolerance


def named_elements(root, tag):
    result = {}
    for node in root.findall(tag):
        name = node.get("name")
        if not name or name != name.strip() or any(character.isspace() for character in name):
            raise ValueError(f"{tag} names must be nonempty and contain no whitespace")
        if name in result:
            raise ValueError(f"Duplicate {tag} name: {name!r}")
        result[name] = node
    return result


def validate_robot(root):
    if root.tag != "robot":
        raise ValueError("Input must be a plain URDF <robot> document")
    if any(isinstance(node.tag, str) and node.tag.startswith("{") for node in root.iter()):
        raise ValueError("Namespaced extensions and unexpanded Xacro are outside this beta's URDF scope")
    links = named_elements(root, "link")
    joints = named_elements(root, "joint")
    if not links:
        raise ValueError("URDF contains no links")
    warnings = []
    for container in list(root.iter("visual")) + list(root.iter("collision")):
        geometries = container.findall("geometry")
        if len(geometries) != 1 or len(geometries[0]) != 1:
            raise ValueError(f"{container.tag} requires exactly one geometry shape")
        shape = geometries[0][0]
        required = {"mesh": (), "box": ("size",), "sphere": ("radius",),
                    "cylinder": ("radius", "length")}
        if shape.tag not in required:
            raise ValueError(f"Unsupported geometry shape: {shape.tag!r}")
        for attribute in required[shape.tag]:
            numbers(shape.get(attribute), 3 if attribute == "size" else 1,
                    f"{shape.tag}.{attribute}")
    for container in list(root.iter("visual")) + list(root.iter("collision")) + list(root.iter("inertial")) + list(root.iter("joint")):
        if len(container.findall("origin")) > 1:
            raise ValueError(f"{container.tag} has multiple origins")
    for joint in joints.values():
        for tag in ("axis", "limit", "mimic"):
            if len(joint.findall(tag)) > 1:
                raise ValueError(f"Joint {joint.get('name')!r} has multiple {tag} elements")
    for node in root.iter():
        for attribute in NUMERIC_ATTRIBUTES.get(node.tag, ()):
            if attribute in node.attrib:
                count = 3 if (node.tag, attribute) in {
                    ("origin", "xyz"), ("origin", "rpy"), ("axis", "xyz"),
                    ("mesh", "scale"), ("box", "size")} else 4 if node.tag == "color" else 1
                values = numbers(node.get(attribute), count, f"{node.tag}.{attribute}")
                if node.tag in {"mesh", "box", "sphere", "cylinder"} and any(value <= 0 for value in values):
                    raise ValueError(f"{node.tag}.{attribute} must be positive")
    parent_of = {}
    children = {name: [] for name in links}
    joint_type = {}
    for name, joint in joints.items():
        kind = joint.get("type")
        if kind not in SUPPORTED_JOINTS:
            raise ValueError(f"Joint {name!r} has unsupported type {kind!r}; supported: {', '.join(sorted(SUPPORTED_JOINTS))}")
        joint_type[name] = kind
        endpoints = []
        for tag in ("parent", "child"):
            nodes = joint.findall(tag)
            if len(nodes) != 1 or nodes[0].get("link") not in links:
                raise ValueError(f"Joint {name!r} requires exactly one existing {tag} link")
            endpoints.append(nodes[0].get("link"))
        parent, child = endpoints
        if parent == child:
            raise ValueError(f"Joint {name!r} connects a link to itself")
        if child in parent_of:
            raise ValueError(f"Link {child!r} has multiple parent joints")
        parent_of[child] = (parent, name)
        children[parent].append(child)
        axis = joint.find("axis")
        if kind != "fixed":
            if axis is None:
                warnings.append(f"Joint {name!r} has no axis; the URDF default 1 0 0 applies")
            else:
                axis_values = numbers(axis.get("xyz"), 3, f"joint {name} axis")
                norm = math.sqrt(sum(value * value for value in axis_values))
                if not math.isfinite(norm) or norm == 0:
                    raise ValueError(f"Joint {name!r} axis must have finite nonzero length")
                if not math.isclose(norm, 1, rel_tol=1e-8):
                    warnings.append(f"Joint {name!r} axis is not unit length; downstream parsers may normalize it")
        limit = joint.find("limit")
        if kind in {"revolute", "prismatic"}:
            missing = [key for key in ("lower", "upper", "effort", "velocity") if limit is None or key not in limit.attrib]
            if missing:
                warnings.append(f"Joint {name!r} lacks limits: {', '.join(missing)}; review before simulator import")
        if limit is not None:
            if "lower" in limit.attrib and "upper" in limit.attrib and scalar(limit, "lower") > scalar(limit, "upper"):
                raise ValueError(f"Joint {name!r} lower limit exceeds upper limit")
            for key in ("effort", "velocity"):
                if key in limit.attrib and scalar(limit, key) < 0:
                    raise ValueError(f"Joint {name!r} {key} limit must be nonnegative")
    roots = [name for name in links if name not in parent_of]
    if len(roots) != 1:
        raise ValueError(f"URDF must have exactly one root link; found {len(roots)}")
    reached, pending = set(), [roots[0]]
    while pending:
        current = pending.pop()
        if current in reached:
            raise ValueError("URDF graph contains a cycle")
        reached.add(current)
        pending.extend(children[current])
    if reached != set(links):
        raise ValueError("URDF graph is disconnected or contains a cycle")
    mimic = {}
    for name, joint in joints.items():
        node = joint.find("mimic")
        if node is not None:
            target = node.get("joint")
            if target not in joints or target == name:
                raise ValueError(f"Joint {name!r} has an invalid mimic target")
            mimic[name] = target
    for name in mimic:
        chain, current = set(), name
        while current in mimic:
            if current in chain:
                raise ValueError("URDF mimic relations contain a cycle")
            chain.add(current)
            current = mimic[current]
    massless_frames = []
    for name, link in links.items():
        parent_joint = parent_of.get(name, (None, None))[1]
        fixed_frame = (not link.findall("visual") and not link.findall("collision") and
                       (parent_joint is None or joint_type[parent_joint] == "fixed"))
        fixed_visual_marker = (bool(link.findall("visual")) and not link.findall("collision") and
                               parent_joint is not None and joint_type[parent_joint] == "fixed")
        inertials = link.findall("inertial")
        if not inertials:
            if fixed_frame:
                massless_frames.append(name)
                continue
            raise ValueError(f"Link {name!r} requires explicit inertial data; only geometry-free fixed frames may be massless")
        if len(inertials) != 1:
            raise ValueError(f"Link {name!r} has multiple inertial blocks")
        inertial = inertials[0]
        masses, tensors = inertial.findall("mass"), inertial.findall("inertia")
        if len(masses) != 1 or len(tensors) != 1:
            raise ValueError(f"Link {name!r} requires one mass and one inertia tensor")
        mass = scalar(masses[0], "value")
        values = [scalar(tensors[0], key) for key in INERTIA_KEYS]
        if mass == 0 and (fixed_frame or fixed_visual_marker) and all(value == 0 for value in values):
            massless_frames.append(name)
            if fixed_visual_marker:
                warnings.append(f"Link {name!r} is a fixed visual-only zero-mass marker; its explicit zero inertia is preserved")
        elif mass <= 0 or not physical_inertia(values):
            raise ValueError(f"Link {name!r} requires positive mass and a physically valid positive-definite inertia tensor")
    return {"root_link": roots[0], "links": len(links), "joints": len(joints),
            "massless_fixed_frames": massless_frames, "warnings": warnings}
