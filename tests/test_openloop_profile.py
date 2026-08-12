"""The open-loop profile must be collision-free.

Open loop means nothing avoids obstacles, so a profile that drives into the
racking or a pallet does not produce a navigation failure — it produces a robot
pinned against a box while its wheels keep turning, which looks exactly like a
catastrophic odometry fault. Two earlier versions of the profile did this and
both cost a full simulation run to diagnose.

Integrating the profile is cheap and settles it before the container starts.
"""

from __future__ import annotations

import math
import re
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
DRIVER = REPO_ROOT / "src" / "amr_bringup" / "amr_bringup" / "nodes" / "openloop_driver.py"

# From simulation/generate_world.py. Duplicated deliberately: if the world moves
# and this is not updated, the test should fail rather than silently track it.
START = (-12.0, 0.0, 0.0)
ROBOT_HALF_LENGTH = 0.35
ROBOT_HALF_WIDTH = 0.275
CIRCUMSCRIBED = math.hypot(ROBOT_HALF_LENGTH, ROBOT_HALF_WIDTH)

RACK_FACE_Y = 1.35  # inner face of the uprights
AISLE_PALLETS = [(-7.1, -5.9, -0.8, 0.0), (3.4, 4.6, -0.05, 0.75)]  # x0, x1, y0, y1
CLEARANCE = 0.15


def load_profile():
    text = DRIVER.read_text(encoding="utf-8")
    block = text[text.index("DEFAULT_PROFILE = ["): text.index("]\n\n\nclass")]
    return [
        (m.group(1), float(m.group(2)), float(m.group(3)), float(m.group(4)))
        for m in re.finditer(r'\("([^"]+)",\s*([-\d.]+),\s*([-\d.]+),\s*([\d.]+)\)', block)
    ]


# Measured turn error: 0.474 rad/s delivered against 0.500 commanded. The
# profile is checked against a heading error at least this large, applied in
# whichever direction is worst, because open loop has no way to correct it.
TURN_ERROR_FRACTION = 0.10


def integrate(profile, dt: float = 0.05, turn_error: float = 0.0):
    """Unicycle model, matching what the diff-drive plugin realises.

    ``turn_error`` scales every commanded angular rate, modelling an open-loop
    turn that under- or over-shoots. A profile that is only clear at
    turn_error = 0 is not clear on the robot.
    """
    x, y, yaw = START
    path = [(x, y, yaw)]
    for _label, v, w, duration in profile:
        w_actual = w * (1.0 + turn_error)
        for _ in range(int(duration / dt)):
            yaw += w_actual * dt
            x += v * dt * math.cos(yaw)
            y += v * dt * math.sin(yaw)
            path.append((x, y, yaw))
    return path


@pytest.fixture(scope="module")
def path():
    """Worst case over plausible turn errors, not the nominal path.

    The nominal path was clear on the attempt that drove into the racking. What
    matters is whether the profile survives the turn error that actually occurs.
    """
    worst = []
    for error in (-TURN_ERROR_FRACTION, 0.0, TURN_ERROR_FRACTION):
        worst.extend(integrate(load_profile(), turn_error=error))
    return worst


def test_profile_parses():
    profile = load_profile()
    assert len(profile) >= 5
    assert all(d > 0 for _, _, _, d in profile)


def test_stays_clear_of_the_racking(path):
    """The circumscribed circle must not reach a rack face."""
    worst = max(abs(y) for _x, y, _yaw in path)
    limit = RACK_FACE_Y - CIRCUMSCRIBED
    assert worst < limit, (
        "profile reaches |y| = %.2f; the robot's circumscribed circle (%.3f m) "
        "would touch the rack face at %.2f m" % (worst, CIRCUMSCRIBED, RACK_FACE_Y)
    )


def test_stays_clear_of_the_aisle_pallets(path):
    """The failure that pinned the robot at x = -7.10 with its wheels spinning."""
    for x, y, _yaw in path:
        for x0, x1, y0, y1 in AISLE_PALLETS:
            near_x = (x0 - CIRCUMSCRIBED) <= x <= (x1 + CIRCUMSCRIBED)
            near_y = (y0 - CIRCUMSCRIBED) <= y <= (y1 + CIRCUMSCRIBED)
            assert not (near_x and near_y), (
                "profile puts the robot at (%.2f, %.2f), within its circumscribed "
                "radius of the pallet spanning x[%.1f, %.1f] y[%.1f, %.1f]"
                % (x, y, x0, x1, y0, y1)
            )


def test_stays_inside_the_building(path):
    for x, y, _yaw in path:
        assert -14.5 < x < 14.5, "profile leaves the building at x=%.2f" % x
        assert abs(y) < 5.5, "profile leaves the aisle at y=%.2f" % y


def test_exercises_every_degree_of_freedom(path):
    """A profile of pure straight lines cannot test a motion model."""
    profile = load_profile()
    assert any(v > 0 and abs(w) < 1e-9 for _, v, w, _ in profile), "no straight run"
    assert any(abs(v) < 1e-9 and abs(w) > 0 for _, v, w, _ in profile), "no in-place rotation"
    assert any(v > 0 and abs(w) > 0 for _, v, w, _ in profile), "no arc"

    total_yaw = sum(abs(w) * d for _, _, w, d in profile)
    assert total_yaw > 2 * math.pi, "less than a full turn of rotational excitation"

    assert any(v < 0 for _, v, _, _ in profile), (
        "no reverse segment; translation should be tested by driving back along "
        "the same heading rather than by turning, because an open-loop turn "
        "cannot be trusted to point the robot where the next segment assumes"
    )


def test_rotations_are_paired_so_errors_cancel(path):
    """Net heading change must be near zero across the whole profile.

    A profile that ends far from its starting heading has relied on turn
    accuracy somewhere, and open loop cannot deliver it.
    """
    profile = load_profile()
    net = sum(w * d for _, _, w, d in profile)
    assert abs(net) < 0.2, "net commanded rotation %.2f rad; pair the turns" % net
