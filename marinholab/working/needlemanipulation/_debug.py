"""
Copyright (C) 2025 Murilo Marques Marinho
(www.murilomarinho.info)

MIT License

Standardized debug output for the needle-manipulation constraint machinery.

Every debug line starts with a fixed-width category tag (for example
``[radius    #0]``) followed by ``key=value`` fields with a fixed numeric
format, so the per-constraint margins printed at a control step are easy to
read and to grep. The available categories are listed in
:data:`CONSTRAINT_CATEGORIES`; the ``verbose`` argument accepted by the
public API has the type :data:`Verbose` (``True`` enables every category,
``False`` disables all of them, and a mapping selects individual
categories).
"""

import math

from termcolor import cprint

#: Constraint families that can be enabled for debug output.
CONSTRAINT_CATEGORIES: tuple[str, ...] = (
    "radius",
    "plane",
    "orientation",
    "insertion",
    "rcm",
)

#: ``verbose`` setting accepted by the public API: ``True`` enables every
#: category, ``False`` disables all of them, and a mapping selects
#: individual categories by name.
Verbose = bool | dict[str, bool]


def normalize_verbose(verbose: Verbose) -> dict[str, bool]:
    """Normalize a ``verbose`` setting into a per-category flag mapping.

    Args:
        verbose: ``True`` enables every category, ``False`` disables all of
            them, and a mapping selects individual categories.

    Returns:
        A mapping with one boolean entry per
        :data:`CONSTRAINT_CATEGORIES` category.

    Raises:
        ValueError: If the mapping contains an unknown category name.
        TypeError: If ``verbose`` is neither a bool nor a mapping.
    """
    if isinstance(verbose, bool):
        return {category: verbose for category in CONSTRAINT_CATEGORIES}
    if isinstance(verbose, dict):
        normalized = {category: False for category in CONSTRAINT_CATEGORIES}
        for category in CONSTRAINT_CATEGORIES:
            normalized[category] = bool(verbose.get(category, False))
        unknown = [key for key in verbose if key not in CONSTRAINT_CATEGORIES]
        if unknown:
            raise ValueError(
                f"Unknown verbose categories {unknown}. "
                f"Expected one of: {', '.join(CONSTRAINT_CATEGORIES)}."
            )
        return normalized
    raise TypeError(
        "verbose must be a bool or a dict mapping category names to booleans."
    )


def _tag(category: str, index: int) -> str:
    """Return the fixed-width debug tag for a category and entity index.

    Args:
        category: One of :data:`CONSTRAINT_CATEGORIES`.
        index: Index of the entity the message refers to (vessel, normal,
            or RCM constraint).

    Returns:
        A tag such as ``[radius    #0]``.
    """
    return f"[{category:<11} #{index}]"


def _violation(description: str) -> None:
    """Print an indented red line describing a violated constraint.

    Args:
        description: Human-readable description of the violation, including
            any measured depth.
    """
    cprint(f"    VIOLATION: {description}", "red")


def debug_radius(
    verbose: dict[str, bool],
    index: int,
    lower: float,
    current: float,
    upper: float,
    margin_upper: float,
    margin_lower: float,
) -> None:
    """Print the radius-constraint debug block for one vessel.

    A negative margin means the corresponding band is violated; the red
    violation line reports the depth in metres.

    Args:
        verbose: Per-category flags from :func:`normalize_verbose`.
        index: Vessel index.
        lower: Lower radius bound (m).
        current: Current needle-to-vessel distance (m).
        upper: Upper radius bound (m).
        margin_upper: Signed margin ``upper^2 - d^2`` (m^2).
        margin_lower: Signed margin ``d^2 - lower^2`` (m^2).
    """
    if not verbose["radius"]:
        return
    tag = _tag("radius", index)
    print(
        f"{tag} lower={lower:.6f} current={current:.6f} "
        f"upper={upper:.6f} (m)"
    )
    print(
        f"{tag} margin_upper={margin_upper:+.6f} "
        f"margin_lower={margin_lower:+.6f} (m^2)"
    )
    if margin_upper < 0.0:
        _violation(
            f"radius upper band exceeded by "
            f"{math.sqrt(-margin_upper):.6f} m"
        )
    if margin_lower < 0.0:
        _violation(
            f"radius lower band breached by "
            f"{math.sqrt(-margin_lower):.6f} m"
        )


def debug_plane(
    verbose: dict[str, bool],
    index: int,
    distance: float,
    margin_upper: float,
    margin_lower: float,
) -> None:
    """Print the plane-constraint debug block for one vessel.

    Args:
        verbose: Per-category flags from :func:`normalize_verbose`.
        index: Vessel index.
        distance: Signed vessel-to-needle-plane distance (m).
        margin_upper: Signed margin to the upper plane bound (m).
        margin_lower: Signed margin to the lower plane bound (m).
    """
    if not verbose["plane"]:
        return
    tag = _tag("plane", index)
    print(f"{tag} distance={distance:+.6f} (m)")
    print(
        f"{tag} margin_upper={margin_upper:+.6f} "
        f"margin_lower={margin_lower:+.6f} (m)"
    )
    if margin_upper < 0.0:
        _violation(f"plane upper bound exceeded by {-margin_upper:.6f} m")
    if margin_lower < 0.0:
        _violation(f"plane lower bound exceeded by {-margin_lower:.6f} m")


def debug_orientation(
    verbose: dict[str, bool],
    index: int,
    current_dot: float,
    margin_upper: float,
    margin_lower: float,
) -> None:
    """Print the orientation-constraint debug block for one vessel normal.

    Args:
        verbose: Per-category flags from :func:`normalize_verbose`.
        index: Normal index.
        current_dot: ``cos`` of the angle between the needle axis and the
            vessel normal (dimensionless).
        margin_upper: Signed upper margin (dimensionless).
        margin_lower: Signed lower margin (dimensionless).
    """
    if not verbose["orientation"]:
        return
    tag = _tag("orientation", index)
    print(f"{tag} current_dot={current_dot:+.6f}")
    print(
        f"{tag} margin_upper={margin_upper:+.6f} "
        f"margin_lower={margin_lower:+.6f}"
    )
    if margin_upper < 0.0:
        _violation(
            f"orientation upper bound exceeded by {-margin_upper:.6f}"
        )
    if margin_lower < 0.0:
        _violation(
            f"orientation lower bound exceeded by {-margin_lower:.6f}"
        )


def debug_insertion(
    verbose: dict[str, bool],
    index: int,
    h: float,
    h_normalized: float,
    angle_deg: float,
    max_angle_deg: float,
    current_dot: float,
    required_dot: float,
    orientation_margin: float,
    insertion_margin: float,
) -> None:
    """Print the insertion-constraint debug block for the first vessel.

    Args:
        verbose: Per-category flags from :func:`normalize_verbose`.
        index: Index of the vessel the insertion constraints refer to.
        h: Signed tip-to-vessel-plane distance (m).
        h_normalized: Normalized depth in ``[0, 1]``.
        angle_deg: Current approach angle (deg).
        max_angle_deg: Maximum permitted angle at depth ``h`` (deg).
        current_dot: ``cos`` of the current approach angle.
        required_dot: ``cos`` of the maximum permitted angle at depth ``h``.
        orientation_margin: ``current_dot - required_dot``.
        insertion_margin: Signed squared tip-to-line margin (m^2).
    """
    if not verbose["insertion"]:
        return
    tag = _tag("insertion", index)
    print(
        f"{tag} h={h:+.6f} (m) h_normalized={h_normalized:+.6f} "
        f"angle={angle_deg:+.2f} (deg) "
        f"max_angle={max_angle_deg:+.2f} (deg)"
    )
    print(
        f"{tag} current_dot={current_dot:+.6f} "
        f"required_dot={required_dot:+.6f} "
        f"orientation_margin={orientation_margin:+.6f}"
    )
    print(f"{tag} insertion_margin={insertion_margin:+.3e} (m^2)")
    if orientation_margin < 0.0:
        _violation(
            f"angular insertion band violated by "
            f"{-orientation_margin:.6f} (dot)"
        )
    if insertion_margin < 0.0:
        _violation(
            f"tip-to-line constraint exceeded by "
            f"{math.sqrt(-insertion_margin):.6f} m"
        )


def debug_rcm(
    verbose: dict[str, bool],
    index: int,
    signed_error: float,
    vfi_gain: float,
) -> None:
    """Print the RCM-constraint debug block for one constraint.

    Args:
        verbose: Per-category flags from :func:`normalize_verbose`.
        index: RCM constraint index.
        signed_error: Signed line-to-point error scaled by the VFI gain
            (m^2).
        vfi_gain: VFI gain applied to the constraint; the violation depth
            is recovered as ``sqrt(-signed_error / vfi_gain)``.
    """
    if not verbose["rcm"]:
        return
    tag = _tag("rcm", index)
    print(f"{tag} signed_error={signed_error:+.6e} (m^2)")
    if signed_error < 0.0:
        if vfi_gain > 0.0:
            depth = math.sqrt(-signed_error / vfi_gain)
        else:
            depth = float("inf")
        _violation(f"RCM safe radius exceeded by {depth:.6f} m")
