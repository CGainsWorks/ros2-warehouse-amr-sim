"""The covariance stamped onto gz-sim's wheel odometry.

gz.msgs.Odometry carries no covariance, so the bridged message arrives with a
zero matrix, which robot_localization clamps to 1e-9 and then trusts
absolutely. amr_bringup's odom_covariance node restores the values the Gazebo
Classic plugin advertised. These tests pin the layout and the values.
"""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from amr_bringup.nodes.odom_covariance import UNMEASURED, odometry_covariance

CONFIG = Path(__file__).resolve().parent.parent / "src" / "amr_bringup" / "config" / "odom_covariance.yaml"


def test_layout_is_row_major_diagonal():
    matrix = odometry_covariance(0.001, 0.002, 0.01)
    assert len(matrix) == 36
    assert [matrix[i * 7] for i in range(6)] == [0.001, 0.002, UNMEASURED, UNMEASURED, UNMEASURED, 0.01]
    assert sum(abs(v) for i, v in enumerate(matrix) if i % 7) == 0.0


@pytest.mark.parametrize("bad", [0.0, -0.1])
def test_zero_or_negative_variance_is_refused(bad):
    """A zero variance is exactly the failure this node exists to prevent."""
    with pytest.raises(ValueError):
        odometry_covariance(bad, 0.001, 0.01)


def test_configured_values_are_the_classic_plugin_values():
    params = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))["odom_covariance"]["ros__parameters"]
    assert (params["covariance_x"], params["covariance_y"], params["covariance_yaw"]) == (0.001, 0.001, 0.01)
    assert params["input_topic"] == "/odom_raw" and params["output_topic"] == "/odom"
