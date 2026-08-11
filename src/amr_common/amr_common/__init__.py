"""Rigid-body maths shared across the stack.

This package exists because three otherwise unrelated consumers need identical
transform code: ``amr_perception`` (moving clouds between frames),
``amr_localisation`` (composing SE(2) poses in the scan matcher) and the offline
scripts in ``analysis/`` (recovering yaw from logged quaternions). The
alternative was three copies of the same quaternion convention, which is how one
of them ends up with a sign error that nobody notices until a cloud appears
mirrored in RViz.

It is a deliberate addition to the layout in the project brief; see
docs/architecture.md.

Nothing here imports ROS.
"""

from .transforms import (
    quat_from_yaw,
    quat_to_matrix,
    se2_compose,
    se2_from_matrix,
    se2_inverse,
    se2_matrix,
    transform_points,
    transform_points_2d,
    wrap_angle,
    yaw_from_quat,
)

__all__ = [
    "quat_from_yaw",
    "quat_to_matrix",
    "se2_compose",
    "se2_from_matrix",
    "se2_inverse",
    "se2_matrix",
    "transform_points",
    "transform_points_2d",
    "wrap_angle",
    "yaw_from_quat",
]
