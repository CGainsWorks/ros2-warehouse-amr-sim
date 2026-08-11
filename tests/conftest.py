"""Shared fixtures.

The ray-traced scan is expensive to produce — a few hundred milliseconds for
28k rays against 117 primitives — so it is built once per session and shared.
Every test that consumes it treats it as read-only.
"""

from __future__ import annotations

import pytest

from amr_perception.core.synthetic import simulate_scan, warehouse_scene

CLUSTER_TOLERANCE = 0.35


@pytest.fixture(scope="session")
def scene():
    return warehouse_scene()


@pytest.fixture(scope="session")
def object_groups(scene):
    return scene.object_groups(tolerance=CLUSTER_TOLERANCE)


@pytest.fixture(scope="session")
def scan(scene):
    """A scan from the middle of the aisle, where both rack runs are in view."""
    return simulate_scan(scene, sensor_xyz=(-3.0, 0.0, 0.9), seed=1)


@pytest.fixture(scope="session")
def far_scan(scene):
    """A scan from the west end, so most structure is at long range."""
    return simulate_scan(scene, sensor_xyz=(-12.0, 0.0, 0.9), seed=2)


@pytest.fixture(scope="session")
def floorplan():
    from amr_localisation.core.synthetic2d import warehouse_floorplan

    return warehouse_floorplan()
