"""Finite decimal URDF serialization, adapted from ACDC4Robot (MIT)."""
from decimal import Decimal, InvalidOperation


NUMERIC_ATTRIBUTES = {
    "origin": ("xyz", "rpy"), "mass": ("value",),
    "inertia": ("ixx", "iyy", "izz", "ixy", "ixz", "iyz"),
    "axis": ("xyz",), "limit": ("lower", "upper", "effort", "velocity"),
    "mesh": ("scale",), "box": ("size",), "sphere": ("radius",),
    "cylinder": ("radius", "length"), "color": ("rgba",),
    "dynamics": ("damping", "friction"), "mimic": ("multiplier", "offset"),
    "safety_controller": ("soft_lower_limit", "soft_upper_limit", "k_position", "k_velocity"),
    "calibration": ("rising", "falling"),
}


def decimal_number(value):
    try:
        number = Decimal(str(value))
    except InvalidOperation as exc:
        raise ValueError(f"Invalid URDF numeric value: {value}") from exc
    if not number.is_finite():
        raise ValueError("URDF numeric values must be finite")
    if number == 0:
        return "0"
    digits = len(number.as_tuple().digits)
    exponent = number.as_tuple().exponent
    if digits + abs(exponent) > 4096:
        raise ValueError("URDF numeric value needs more than 4096 decimal characters")
    result = format(number, "f")
    return result.rstrip("0").rstrip(".") if "." in result else result


def normalize_numbers(root):
    for node in root.iter():
        for attribute in NUMERIC_ATTRIBUTES.get(node.tag, ()):
            if attribute in node.attrib:
                node.set(attribute, " ".join(decimal_number(token) for token in node.get(attribute).split()))
    return root
