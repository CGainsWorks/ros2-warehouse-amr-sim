"""The README's simulated localisation numbers must match the committed reference.

Simulated runs are not bit reproducible across machines, so CI cannot rerun
them and diff. What it can do is make sure the numbers a reader sees are the
ones that were recorded: results/reference/gazebo_localisation.json holds one
recorded open-loop run and one recorded mission, and the README's tables must
be exactly what analysis/results_reference.py renders from it.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "analysis"))

import results_reference as ref  # noqa: E402


@pytest.fixture(scope="module")
def reference():
    return json.loads(ref.REFERENCE.read_text(encoding="utf-8"))


def test_reference_is_labelled_simulated(reference):
    assert reference["simulated"] is True
    assert "gz-sim" in reference["environment"]
    assert set(reference["runs"]) == set(ref.RUNS)


def test_reference_holds_no_host_paths():
    text = ref.REFERENCE.read_text(encoding="utf-8")
    for marker in ("C:\\", "/Users/", "/home/", "/mnt/", "/ws/"):
        assert marker not in text, "host path %r in the committed reference" % marker


@pytest.mark.parametrize("run", ref.RUNS)
def test_readme_table_matches_the_reference(reference, run):
    blocks = ref.readme_blocks(ref.README.read_text(encoding="utf-8"))
    assert run in blocks, "README has no <!-- gz-results:%s --> block" % run
    expected = ref.render(reference)[run]
    assert blocks[run] == expected, (
        "README's %s table disagrees with %s. Regenerate it with "
        "`python3 analysis/results_reference.py render`.\nREADME:\n%s\nexpected:\n%s"
        % (run, ref.REFERENCE.relative_to(REPO_ROOT).as_posix(), blocks[run], expected)
    )


def test_render_formats_every_source(reference):
    for run in ref.RUNS:
        table = ref.render_table(reference["runs"][run])
        for _key, label, _error, _audit in ref.SOURCES:
            assert "| %s |" % label in table


def test_summary_rounds_and_computes_path_ratio():
    error = {"ate_rmse_m": 0.12345, "ate_max_m": 0.5, "yaw_rmse_deg": 1.234,
             "nees_mean": 2.049, "n_jumps_over_0p5m": 0}
    audit = {"amcl_pose": {"path_length_m": 101.0}}
    out = ref._source_summary(error, audit, "amcl_pose", 100.0)
    assert out == {"ate_rmse_m": 0.123, "ate_max_m": 0.5, "yaw_rmse_deg": 1.23, "nees_mean": 2.05,
                   "jumps_over_0p5m": 0, "path_vs_truth_pct": 1.0}
