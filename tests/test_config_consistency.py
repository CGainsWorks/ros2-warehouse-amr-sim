"""Cross-package configuration consistency.

Several numbers in this stack appear in more than one file and have to agree.
None of these are enforced by any framework, and when they drift the symptom is
never "the config is inconsistent" — it is a robot that overshoots every goal,
or a planner that produces paths the controller refuses to follow, or a costmap
that quietly stops seeing obstacles at range. Every assertion here corresponds
to a specific failure mode, named in the test.

These tests read the shipped configuration files. They are the reason a change
to, say, the wheel radius in the URDF cannot silently invalidate the DWB
acceleration limits.
"""

from __future__ import annotations

import re
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
SRC = REPO_ROOT / "src"

NAV2_PARAMS = SRC / "amr_navigation" / "config" / "nav2_params.yaml"
PERCEPTION_PARAMS = SRC / "amr_perception" / "config" / "perception.yaml"
EKF_PARAMS = SRC / "amr_localisation" / "config" / "ekf.yaml"
AMCL_PARAMS = SRC / "amr_localisation" / "config" / "amcl.yaml"
URDF = SRC / "amr_description" / "urdf" / "amr.urdf.xacro"
GAZEBO_XACRO = SRC / "amr_description" / "urdf" / "amr.gazebo.xacro"
ZONES = REPO_ROOT / "simulation" / "zones" / "warehouse_zones.yaml"
WAYPOINTS = SRC / "amr_bringup" / "config" / "mission_waypoints.yaml"
MAP_YAML = REPO_ROOT / "simulation" / "maps" / "warehouse.yaml"
GZ_BRIDGE = SRC / "amr_bringup" / "config" / "gz_bridge.yaml"
WORLD = REPO_ROOT / "simulation" / "worlds" / "warehouse.world"


def _load_yaml(path: Path) -> dict:
    text = path.read_text(encoding="utf-8")
    # nav2_params.yaml uses $(find-pkg-share ...), which the launch system
    # substitutes but PyYAML does not understand as anything special. It parses
    # fine as a string; nothing here needs to resolve it.
    return yaml.safe_load(text)


def _xacro_property(name: str) -> float:
    """Read a <xacro:property name="x" value="y"/> from the URDF."""
    text = URDF.read_text(encoding="utf-8")
    match = re.search(r'<xacro:property\s+name="%s"\s+value="([^"]+)"' % re.escape(name), text)
    assert match, "property %r not found in %s" % (name, URDF.name)
    return float(match.group(1))


def _gazebo_scalar(tag: str) -> float:
    text = GAZEBO_XACRO.read_text(encoding="utf-8")
    match = re.search(r"<%s>([^<]+)</%s>" % (tag, tag), text)
    assert match, "tag %r not found in %s" % (tag, GAZEBO_XACRO.name)
    return float(match.group(1))


@pytest.fixture(scope="module")
def nav2():
    return _load_yaml(NAV2_PARAMS)


@pytest.fixture(scope="module")
def dwb(nav2):
    return nav2["controller_server"]["ros__parameters"]["FollowPath"]


@pytest.fixture(scope="module")
def local_costmap(nav2):
    return nav2["local_costmap"]["local_costmap"]["ros__parameters"]


@pytest.fixture(scope="module")
def global_costmap(nav2):
    return nav2["global_costmap"]["global_costmap"]["ros__parameters"]


class TestRobotGeometry:
    def test_wheel_separation_matches_between_urdf_and_gazebo(self):
        """A mismatch makes the plugin's odometry wrong in yaw by a fixed ratio,
        which the EKF cannot distinguish from a persistent gyro bias."""
        assert _xacro_property("wheel_separation") == _gazebo_scalar("wheel_separation")

    def test_wheel_radius_matches_the_urdf(self):
        """A mismatch scales all reported distance, which looks exactly like a
        slipping wheel and will be chased as one."""
        assert _gazebo_scalar("wheel_radius") == pytest.approx(_xacro_property("wheel_radius"))

    def test_costmap_footprint_covers_the_widest_part_of_the_robot(
        self, local_costmap, global_costmap
    ):
        """A footprint smaller than the robot lets it plan into the racking.

        The widest point is NOT the chassis. The drive wheels are 0.05 m wide
        centred at y = +/-0.25, so they reach y = +/-0.275, 25 mm proud of the
        0.50 m chassis. A footprint set from base_width alone understates the
        robot by exactly the parts that stick out furthest — which are the ones
        that would clip a racking upright.
        """
        length = _xacro_property("base_length")
        chassis_half_width = _xacro_property("base_width") / 2.0
        wheel_half_width = _xacro_property("wheel_separation") / 2.0 + _xacro_property("wheel_width") / 2.0
        required_half_width = max(chassis_half_width, wheel_half_width)

        for costmap in (local_costmap, global_costmap):
            footprint = yaml.safe_load(costmap["footprint"])
            xs = [abs(p[0]) for p in footprint]
            ys = [abs(p[1]) for p in footprint]
            assert max(xs) == pytest.approx(length / 2.0), "footprint length disagrees with the URDF"
            assert max(ys) >= required_half_width - 1e-9, (
                "footprint half-width %.3f is narrower than the robot's widest point %.3f "
                "(wheels reach beyond the chassis)" % (max(ys), required_half_width)
            )

    def test_both_costmaps_use_the_same_footprint(self, local_costmap, global_costmap):
        assert local_costmap["footprint"] == global_costmap["footprint"]


class TestDynamics:
    def test_dwb_acceleration_matches_the_base(self, dwb):
        """DWB planning more deceleration than the wheels can deliver overshoots
        every goal and clips every corner."""
        linear_accel = _gazebo_scalar("max_linear_acceleration")  # m/s^2
        assert _gazebo_scalar("min_linear_acceleration") == pytest.approx(-linear_accel)
        assert dwb["acc_lim_x"] == pytest.approx(linear_accel), (
            "acc_lim_x is %.2f m/s^2 but the base can only do %.2f"
            % (dwb["acc_lim_x"], linear_accel)
        )

    def test_deceleration_mirrors_acceleration(self, dwb):
        assert dwb["decel_lim_x"] == pytest.approx(-dwb["acc_lim_x"])
        assert dwb["decel_lim_theta"] == pytest.approx(-dwb["acc_lim_theta"])

    def test_velocity_smoother_agrees_with_dwb(self, nav2, dwb):
        """The smoother sits between DWB and the base. Limits below DWB's make
        the robot fail to follow its own plans; above them do nothing."""
        smoother = nav2["velocity_smoother"]["ros__parameters"]
        assert smoother["max_velocity"][0] == pytest.approx(dwb["max_vel_x"])
        assert smoother["max_velocity"][2] == pytest.approx(dwb["max_vel_theta"])
        assert smoother["max_accel"][0] == pytest.approx(dwb["acc_lim_x"])
        assert smoother["max_accel"][2] == pytest.approx(dwb["acc_lim_theta"])

    def test_differential_drive_has_no_lateral_motion(self, dwb):
        assert dwb["max_vel_y"] == 0.0
        assert dwb["acc_lim_y"] == 0.0

    def test_rollout_is_long_enough_to_stop_within(self, dwb):
        """Every simulated trajectory must be haltable inside itself, or DWB can
        commit to a path it cannot abort."""
        stopping_distance = dwb["max_vel_x"] ** 2 / (2 * dwb["acc_lim_x"])
        rollout = dwb["max_vel_x"] * dwb["sim_time"]
        assert rollout > stopping_distance, (
            "rollout %.2f m is shorter than the %.2f m stopping distance"
            % (rollout, stopping_distance)
        )


class TestLintConfiguration:
    """The lint settings live in .flake8. Anything that passes them again on a
    command line silently overrides that file."""

    def test_nothing_overrides_the_flake8_config(self):
        """CI passed --max-line-length=110 over a .flake8 that says 120, so the
        CI job was failing on 24 pre-existing lines while `make lint` reported
        success -- it passed the same override *and* ended in `|| true`, so it
        could not fail at all. The config file's own comment says this cannot
        happen, which is how it went unnoticed."""
        for path in (REPO_ROOT / "Makefile",
                     REPO_ROOT / ".github" / "workflows" / "ci.yml"):
            for line in path.read_text(encoding="utf-8").splitlines():
                if line.lstrip().startswith("#"):
                    continue
                if "flake8" not in line:
                    continue
                for flag in ("--max-line-length", "--extend-ignore", "--ignore"):
                    assert flag not in line, (
                        "%s passes %s to flake8, overriding .flake8: %s"
                        % (path.name, flag, line.strip())
                    )

    def test_the_lint_target_can_fail(self):
        """`|| true` on a lint target means it reports success no matter what."""
        makefile = (REPO_ROOT / "Makefile").read_text(encoding="utf-8")
        lint_body = makefile.split("lint:", 1)[1].split("\n.PHONY", 1)[0]
        assert "|| true" not in lint_body


class TestOfflineDependencies:
    """The offline half of this repository is what CI actually gates on, and it
    has exactly one dependency list."""

    def test_ci_installs_from_requirements(self):
        """`cache: pip` in setup-python hashes **/requirements.txt. With no such
        file anywhere in the tree it does not warn, it fails the job in a few
        seconds, before any check has run -- which reads as "CI is broken" with
        nothing in the log about linting or tests."""
        requirements = REPO_ROOT / "requirements.txt"
        assert requirements.exists(), "setup-python's pip cache needs this file"

        workflow = (REPO_ROOT / ".github" / "workflows"
                    / "ci.yml").read_text(encoding="utf-8")
        assert "-r requirements.txt" in workflow

    def test_requirements_covers_what_the_offline_checks_import(self):
        """A missing entry here fails in CI as an ImportError two steps later."""
        requirements = (REPO_ROOT / "requirements.txt").read_text(encoding="utf-8")
        listed = {
            line.split("=")[0].split(">")[0].split("<")[0].strip().lower()
            for line in requirements.splitlines()
            if line.strip() and not line.startswith("#")
        }
        for package in ("numpy", "scipy", "pyyaml", "pytest", "flake8"):
            assert package in listed, "requirements.txt is missing %s" % package

    def test_the_readme_quick_start_installs_the_same_things(self):
        """The README told people to install a hand-written list that CI did not
        use, so the two could drift without either being wrong on its own."""
        readme = (REPO_ROOT / "README.md").read_text(encoding="utf-8")
        for line in readme.splitlines():
            if line.strip().startswith("pip install"):
                assert "-r requirements.txt" in line, (
                    "README pins its own dependency list: %s" % line.strip()
                )


class TestFaultInjection:
    """The fault harness is the only part of this repository whose job is to
    break the robot, so its own failure modes are quiet ones."""

    def test_every_scheduled_fault_has_an_implementation(self):
        """An unknown fault type is logged and skipped at runtime, so a typo in
        the schedule reads as "the robot handled it" in the matrix."""
        schedule = _load_yaml(SRC / "amr_bringup" / "config" / "fault_schedule.yaml")
        source = (SRC / "amr_bringup" / "amr_bringup" / "nodes"
                  / "fault_injector.py").read_text(encoding="utf-8")

        for fault in schedule["faults"]:
            kind = fault["type"]
            assert '"%s"' % kind in source, (
                "fault_schedule.yaml schedules %r, which fault_injector.py does "
                "not implement. At runtime that is logged and skipped, so the "
                "recovery matrix would show a fault that never fired." % kind
            )

    def test_faults_are_ordered_and_leave_room_to_recover(self):
        """Two faults inside one recovery cycle cannot be told apart in the
        matrix — whichever recovery fires gets attributed to the later one."""
        schedule = _load_yaml(SRC / "amr_bringup" / "config" / "fault_schedule.yaml")
        times = [float(f["at"]) for f in schedule["faults"]]
        assert times == sorted(times), "faults must be in time order"
        for earlier, later in zip(times, times[1:]):
            assert later - earlier >= 15.0, (
                "faults at %.0fs and %.0fs are closer than one recovery cycle"
                % (earlier, later)
            )

    def test_the_dropout_relay_is_not_in_the_path_by_default(self):
        """The localisation figures in the docs were measured with the LiDAR
        wired straight to the stack. If the relay defaulted on, every run would
        carry a Python hop in front of AMCL and the numbers would not be
        comparable."""
        launch = (SRC / "amr_bringup" / "launch"
                  / "simulation.launch.py").read_text(encoding="utf-8")
        declaration = re.search(
            r'DeclareLaunchArgument\(\s*"use_scan_relay",\s*default_value="(\w+)"',
            launch,
        )
        assert declaration is not None, "use_scan_relay is not declared"
        assert declaration.group(1) == "false"

        urdf = (SRC / "amr_description" / "urdf"
                / "amr.urdf.xacro").read_text(encoding="utf-8")
        assert '<xacro:arg name="scan_topic" default="scan"/>' in urdf

    def test_each_fault_is_measured_in_its_own_run(self):
        """Firing the whole schedule into one mission produces numbers that
        cannot be read: a fault that ends the run hides everything after it, and
        a fault that lands on an already-broken robot has its recovery latency
        confounded with the previous fault still unwinding. Both happened."""
        script = (REPO_ROOT / "scripts" / "run_faults.sh").read_text(encoding="utf-8")
        assert "--runs" in script, "the matrix must be built from per-fault runs"
        assert 'for index in "${!FAULT_TYPES[@]}"' in script

    def test_the_readiness_check_can_time_out(self):
        """`ros2 topic echo --once` blocks forever on a topic that never
        appears, so an unwrapped call makes the startup deadline below it
        unreachable. The harness sat in that state for eighteen minutes."""
        for name in ("run_faults.sh", "run_evaluation.sh", "run_openloop.sh"):
            script_path = REPO_ROOT / "scripts" / name
            if not script_path.exists():
                continue
            script = script_path.read_text(encoding="utf-8")
            for line in script.splitlines():
                if line.lstrip().startswith("#"):
                    continue
                if "ros2 topic echo" in line and "--once" in line:
                    assert "timeout " in line, (
                        "%s waits on `ros2 topic echo --once` without a timeout, "
                        "so its startup deadline can never fire: %s"
                        % (name, line.strip())
                    )

    def test_the_dropout_is_not_implemented_by_stopping_the_simulator(self):
        """It was, once. Suspending gzserver stops the clock, and under
        use_sim_time every node in the stack stops with it, so nothing observes
        a dropout at all."""
        script = (REPO_ROOT / "scripts" / "run_faults.sh").read_text(encoding="utf-8")
        assert "pkill -STOP" not in script
        assert "use_scan_relay:=true" in script


class TestCostmaps:
    def test_inflation_is_identical_in_both_costmaps(self, local_costmap, global_costmap):
        """Differing inflation makes the planner produce paths the controller
        will not follow, and the robot oscillates between replan and recovery
        with nothing obviously wrong in either log."""
        a = local_costmap["inflation_layer"]
        b = global_costmap["inflation_layer"]
        assert a["inflation_radius"] == b["inflation_radius"]
        assert a["cost_scaling_factor"] == b["cost_scaling_factor"]

    def test_inflation_leaves_a_navigable_corridor(self, local_costmap):
        """The binding constraint in a 2.8 m aisle.

        Racking faces are at |y| = 1.40. If the inflated bands from both sides
        meet, every plan runs through non-zero cost and the gradient stops
        carrying any information about which side to prefer.
        """
        radius = local_costmap["inflation_layer"]["inflation_radius"]
        aisle_half_width = 1.40
        robot_half_width = _xacro_property("base_width") / 2.0

        free_half_width = aisle_half_width - radius
        assert free_half_width > robot_half_width, (
            "inflation_radius %.2f leaves only %.2f m of free half-width for a "
            "robot %.2f m wide" % (radius, free_half_width, robot_half_width * 2)
        )

    def test_inflation_leaves_a_lane_past_an_obstacle_in_the_aisle(self, local_costmap):
        """The constraint that actually stopped the robot, and the reason this
        test exists.

        A person standing in the middle of the aisle is inflated from one side,
        and the racking is inflated from the other. If the two inflated regions
        meet there is no zero-cost route past the obstacle at all, NavFn
        eventually reports "failed to create plan", and the mission aborts. At
        inflation_radius 0.65 the aisle was closed by 0.15 m and none of the
        other tests here noticed.

        Obstacle positions come from the world generator rather than from a
        constant, so moving a pallet or a person re-derives the bound.
        """
        import sys

        sys.path.insert(0, str(REPO_ROOT / "simulation"))
        import generate_world as world

        radius = local_costmap["inflation_layer"]["inflation_radius"]
        aisle_half_width = world.AISLE_HALF_WIDTH
        localisation_rmse = 0.077  # measured; docs/localisation-accuracy.md

        # (half-width across the aisle, centre y) for everything left in the aisle.
        obstacles = [("person", 0.25, y) for _, y in world.PEOPLE]
        obstacles += [("pallet", 0.8 / 2.0, y) for _, y in world.FLOOR_PALLETS]

        # And wherever the fault injector *moves* one to. block_aisle originally
        # dropped a pallet dead centre, leaving a 0.06 m lane, so the "replan
        # around the obstacle" fault was really an impassable-aisle fault and
        # the robot was marked as failing to recover from it.
        schedule = _load_yaml(SRC / "amr_bringup" / "config" / "fault_schedule.yaml")
        obstacles += [
            ("pallet dropped by %s" % fault["type"], 0.8 / 2.0, float(fault["y"]))
            for fault in schedule["faults"]
            if fault["type"] == "block_aisle"
        ]

        for kind, half_width, centre in obstacles:
            # A robot passes on whichever side is wider, so the binding
            # constraint is the wider gap, not the nearer rack face.
            north = aisle_half_width - (centre + half_width)
            south = (centre - half_width) + aisle_half_width
            free = max(north, south)
            lane = free - 2 * radius
            assert lane >= 2 * localisation_rmse, (
                "a %s %.2f m wide at y=%+.2f leaves a zero-cost lane of %.3f m at "
                "inflation_radius %.2f, which is under twice the measured "
                "localisation RMSE of %.3f m. The planner has no route it can "
                "hold. inflation_radius must be below %.3f here."
                % (kind, 2 * half_width, centre, lane, radius, localisation_rmse,
                   (free - 2 * localisation_rmse) / 2.0)
            )

    def test_inflation_exceeds_the_circumscribed_radius(self, local_costmap):
        """Below the circumscribed radius the robot's own corners can reach
        cells the inflation says are safe."""
        import math

        radius = local_costmap["inflation_layer"]["inflation_radius"]
        circumscribed = math.hypot(_xacro_property("base_length") / 2.0,
                                   _xacro_property("base_width") / 2.0)
        assert radius > circumscribed

    def test_clearing_range_exceeds_marking_range(self, local_costmap, global_costmap):
        """If they are equal, a cell marked at the sensor's limit can never be
        cleared and the costmap accumulates permanent phantom obstacles."""
        for costmap in (local_costmap, global_costmap):
            layer = costmap["obstacle_layer"]
            for source in str(layer["observation_sources"]).split():
                config = layer[source]
                if not config.get("clearing", False):
                    continue
                assert config["raytrace_max_range"] > config["obstacle_max_range"], (
                    "%s clears to %.1f m but marks to %.1f m"
                    % (source, config["raytrace_max_range"], config["obstacle_max_range"])
                )

    def test_segmentation_source_does_not_clear(self, local_costmap):
        """The segmented cloud has had the ground removed, so it carries no
        free-space evidence. Ray-tracing through it would clear cells the sensor
        never saw as empty."""
        layer = local_costmap["obstacle_layer"]
        assert layer["segmentation"]["clearing"] is False
        assert layer["segmentation"]["marking"] is True

    def test_local_costmap_contains_the_rollout(self, local_costmap, dwb):
        """Trajectories running off the edge of the costmap get scored against
        unknown space rather than against obstacles."""
        half_size = min(local_costmap["width"], local_costmap["height"]) / 2.0
        rollout = dwb["max_vel_x"] * dwb["sim_time"]
        needed = rollout + local_costmap["inflation_layer"]["inflation_radius"]
        assert half_size > needed, "local costmap half-size %.1f m < required %.1f m" % (half_size, needed)

    def test_global_costmap_covers_the_warehouse(self, global_costmap):
        assert global_costmap["width"] >= 32
        assert global_costmap["height"] >= 14

    def test_global_costmap_resolution_matches_the_map(self, global_costmap):
        """A mismatch makes the static layer resample the map every update."""
        map_meta = _load_yaml(MAP_YAML)
        assert global_costmap["resolution"] == pytest.approx(map_meta["resolution"])

    def test_unknown_space_is_not_planned_through(self, global_costmap, nav2):
        """The warehouse is fully mapped, so unknown space means a mapping gap
        or a localisation failure. Planning confidently through either is wrong."""
        assert global_costmap["track_unknown_space"] is False
        assert nav2["planner_server"]["ros__parameters"]["GridBased"]["allow_unknown"] is False


class TestPerceptionAndNavigationAgree:
    def test_clustering_parameters_match_across_both_perception_nodes(self):
        """The zone publisher re-clusters the segmentation output. If it uses a
        different tolerance, the cluster ids it reports do not correspond to the
        ones the segmentation node published on the same stamp."""
        params = _load_yaml(PERCEPTION_PARAMS)
        seg = params["/**/segmentation_node"]["ros__parameters"]
        zone = params["/**/zone_occupancy_node"]["ros__parameters"]
        for key in ("cluster_tolerance", "cluster_min_points", "cluster_voxel_size"):
            assert seg[key] == zone[key], "%s differs between the two perception nodes" % key

    def test_voxel_size_is_below_the_clustering_tolerance(self):
        """Above it, downsampling severs the connections clustering must find."""
        seg = _load_yaml(PERCEPTION_PARAMS)["/**/segmentation_node"]["ros__parameters"]
        assert seg["cluster_voxel_size"] < seg["cluster_tolerance"]

    def test_costmap_height_band_matches_the_perception_filter(self, local_costmap):
        """The costmap must not be given returns the segmentation already
        discarded, nor discard ones it kept."""
        seg = _load_yaml(PERCEPTION_PARAMS)["/**/segmentation_node"]["ros__parameters"]
        source = local_costmap["obstacle_layer"]["segmentation"]
        assert source["min_obstacle_height"] >= seg["cluster_min_height"] - 1e-9
        assert source["max_obstacle_height"] <= seg["max_obstacle_height"]

    def test_costmap_topic_matches_the_segmentation_publisher(self, local_costmap):
        assert local_costmap["obstacle_layer"]["segmentation"]["topic"] == "/segmentation_node/obstacles"


class TestLocalisation:
    def test_ekf_owns_odom_to_base_link(self):
        """Two publishers on one transform make the TF tree flip between them."""
        ekf = _load_yaml(EKF_PARAMS)["ekf_filter_node"]["ros__parameters"]
        assert ekf["publish_tf"] is True
        assert ekf["world_frame"] == ekf["odom_frame"], (
            "world_frame must be odom so the EKF publishes odom->base_link; "
            "AMCL owns map->odom"
        )
        # gz-sim's DiffDrive always publishes a transform; it must go to a topic
        # the bridge does not carry, or odom -> base_footprint has two owners.
        tf_topic = re.search(r"<tf_topic>([^<]+)</tf_topic>", GAZEBO_XACRO.read_text(encoding="utf-8")).group(1)
        bridged = {entry["gz_topic_name"] for entry in _load_yaml(GZ_BRIDGE)}
        assert tf_topic not in bridged and "/tf" not in bridged, (
            "the DiffDrive transform is bridged into ROS; the EKF must be the only odom TF publisher"
        )

    def test_ekf_base_frame_is_the_urdf_root(self):
        """The EKF must publish odom -> the URDF's ROOT link.

        Regression guard for a bug that cost a full simulation run. The URDF
        roots at base_footprint and robot_state_publisher owns the fixed edge
        base_footprint -> base_link. Setting the EKF's base_link_frame to
        base_link makes it publish odom -> base_link as well, giving base_link
        two parents. That is not a tree, so tf2 splits into two disconnected
        components and every lookup spanning them fails.

        What makes it worth a test is how badly the symptom misdirects: AMCL
        complains it cannot connect base_footprint to lidar_2d_link, never
        publishes map -> odom, the map frame ceases to exist, and the
        perception nodes go silent. None of that points at a single wrong word
        in ekf.yaml.
        """
        text = URDF.read_text(encoding="utf-8")
        # The root is the link that is never a joint's child. Only the
        # non-macro joints are visible without expanding xacro, which is
        # sufficient: the root is always at the top level.
        children = set(re.findall(r'<child\s+link="([^"]+)"', text))
        parents = set(re.findall(r'<parent\s+link="([^"]+)"', text))
        roots = parents - children
        assert len(roots) == 1, "expected exactly one URDF root, found %s" % sorted(roots)
        root = roots.pop()

        ekf = _load_yaml(EKF_PARAMS)["ekf_filter_node"]["ros__parameters"]
        assert ekf["base_link_frame"] == root, (
            "EKF publishes odom -> %r but the URDF root is %r; %r would then have "
            "two parents and the TF tree would split"
            % (ekf["base_link_frame"], root, ekf["base_link_frame"])
        )

    def test_base_frame_agrees_across_ekf_amcl_and_gazebo(self):
        """All three must name the same frame or the tree splits or drifts."""
        ekf = _load_yaml(EKF_PARAMS)["ekf_filter_node"]["ros__parameters"]
        amcl = _load_yaml(AMCL_PARAMS)["amcl"]["ros__parameters"]
        gazebo = GAZEBO_XACRO.read_text(encoding="utf-8")
        plugin_frame = re.search(r"<child_frame_id>([^<]+)</child_frame_id>", gazebo).group(1)

        assert ekf["base_link_frame"] == amcl["base_frame_id"] == plugin_frame, (
            "base frame disagrees: EKF=%r AMCL=%r gazebo plugin=%r"
            % (ekf["base_link_frame"], amcl["base_frame_id"], plugin_frame)
        )

    def test_ekf_is_two_d(self):
        ekf = _load_yaml(EKF_PARAMS)["ekf_filter_node"]["ros__parameters"]
        assert ekf["two_d_mode"] is True

    def test_ekf_fuses_three_sources(self):
        ekf = _load_yaml(EKF_PARAMS)["ekf_filter_node"]["ros__parameters"]
        assert "odom0" in ekf and "imu0" in ekf and "odom1" in ekf

    def test_wheel_odometry_is_fused_differentially_not_as_raw_twist(self):
        """Fuse the wheel pose differentially, not the twist.

        Kept from Gazebo Classic, where it was forced: gazebo_ros_diff_drive
        computes its ENCODER forward velocity as
        sqrt(dx^2 + dy^2)/dt, a magnitude that is never negative. The pose it
        integrates is correctly signed; the twist is not. Fusing the twist
        integrates every reverse as a forward: on an open-loop profile driving
        3.5 m out and back, ground truth and the wheel pose both returned to the
        start (net 1.2 m and 1.34 m) while the EKF ran to a net 11.7 m with a
        perfectly correct path length, because it was integrating |v|.

        Differential mode differentiates the pose, so this is still velocity
        fusion and it keeps the sign. gz-sim's DiffDrive publishes a signed
        twist, but the localisation figures were tuned with pose fusion, and
        keeping it means the EKF is right whichever simulator or base driver
        feeds it.
        """
        ekf = _load_yaml(EKF_PARAMS)["ekf_filter_node"]["ros__parameters"]
        config = ekf["odom0_config"]
        assert ekf["odom0_differential"] is True, (
            "wheel odometry must be fused differentially; the plugin's twist is unsigned"
        )
        assert config[0] is True and config[1] is True, "differential mode needs x and y"
        assert config[5] is True, "differential mode needs yaw"
        assert not any(config[6:]), (
            "do not fuse the plugin's twist: its linear.x is a magnitude and loses "
            "the sign of reverse motion"
        )

    def test_scan_odometry_is_differential(self):
        """A dead-reckoned source whose absolute value drifts must be fused as
        increments, or it competes with AMCL for the absolute-position job."""
        ekf = _load_yaml(EKF_PARAMS)["ekf_filter_node"]["ros__parameters"]
        assert ekf["odom1_differential"] is True

    def test_imu_does_not_contribute_absolute_yaw(self):
        """Gazebo's IMU will happily publish simulator-derived ground-truth
        orientation. Fusing it makes the localisation results unachievable on
        hardware and therefore worthless."""
        ekf = _load_yaml(EKF_PARAMS)["ekf_filter_node"]["ros__parameters"]
        assert ekf["imu0_config"][5] is False, "IMU absolute yaw must not be fused"
        assert ekf["imu0_config"][11] is True, "IMU should contribute yaw rate"

    def test_imu_linear_acceleration_is_not_fused(self):
        """A biased accelerometer integrates straight into position drift.

        robot_localization does not estimate accelerometer bias, and the
        simulated IMU carries 0.1 m/s^2 of it. Fusing ax made the filter
        integrate that bias into a phantom forward velocity: on an open-loop
        profile that drives 3.5 m out and back, true net displacement 1.2 m and
        wheel odometry 1.34 m, the EKF reported 11.6 m and an ATE of 6.7 m
        against wheel odometry's 0.49 m — six times worse than its own input.
        A wheel encoder measures forward velocity directly and far better.
        """
        ekf = _load_yaml(EKF_PARAMS)["ekf_filter_node"]["ros__parameters"]
        assert ekf["imu0_config"][12] is False, (
            "IMU linear acceleration is fused; without bias estimation this "
            "integrates the accelerometer bias into position"
        )

    def test_amcl_consumes_the_ekf_output_frame(self):
        amcl = _load_yaml(AMCL_PARAMS)["amcl"]["ros__parameters"]
        ekf = _load_yaml(EKF_PARAMS)["ekf_filter_node"]["ros__parameters"]
        assert amcl["odom_frame_id"] == ekf["odom_frame"]
        assert amcl["global_frame_id"] == "map"

    def test_amcl_laser_range_is_within_the_sensor(self):
        amcl = _load_yaml(AMCL_PARAMS)["amcl"]["ros__parameters"]
        text = GAZEBO_XACRO.read_text(encoding="utf-8")
        # The 2D LiDAR block's <max> is the first range maximum in the file.
        sensor_max = float(re.search(r"<max>([\d.]+)</max>", text).group(1))
        assert amcl["laser_max_range"] <= sensor_max

    def test_wheel_odometry_is_not_ground_truth(self):
        """The stack's /odom must come from the wheels, never from the simulator's pose.

        Under Gazebo Classic this was odometry_source, where 1 meant WORLD and
        silently published ground truth as odometry; this file once set it,
        with a comment claiming it meant ENCODER. gz-sim has no such switch:
        DiffDrive integrates the wheel joints, and ground truth comes from a
        separate OdometryPublisher system. What can still go wrong is wiring,
        so this pins it: DiffDrive feeds odom_covariance, which publishes /odom,
        and the ground-truth topic is a different one that only pose_logger reads.
        """
        text = GAZEBO_XACRO.read_text(encoding="utf-8")
        odom_topics = re.findall(r"<odom_topic>([^<]+)</odom_topic>", text)
        assert len(odom_topics) == 2, "expected DiffDrive and OdometryPublisher, found %s" % odom_topics
        diff_drive = text.split("gz::sim::systems::DiffDrive")[1].split("</plugin>")[0]
        wheel_topic = re.search(r"<odom_topic>([^<]+)</odom_topic>", diff_drive).group(1)
        ground_truth = [t for t in odom_topics if t != wheel_topic]
        assert ground_truth == ["/ground_truth/pose"]

        relay = _load_yaml(SRC / "amr_bringup" / "config" / "odom_covariance.yaml")
        relay = relay["odom_covariance"]["ros__parameters"]
        assert relay["input_topic"] == wheel_topic and relay["output_topic"] == "/odom"
        ekf = _load_yaml(EKF_PARAMS)["ekf_filter_node"]["ros__parameters"]
        assert ekf["odom0"] == "/odom"

    def test_casters_clear_the_floor(self):
        """Casters must sit above the floor so the drive wheels carry the load.

        With caster bottoms and wheel bottoms both at exactly z = 0, the load
        split is numerically indeterminate: the contact solver resolves the stiff sphere-plane
        contacts first, the drive wheels lose normal force, and the robot sits
        still while its wheels spin at exactly the commanded rate. Every plugin
        reports success and no log line is wrong.
        """
        clearance = _xacro_property("caster_clearance")
        assert clearance > 0.0, (
            "caster_clearance is %r; with zero clearance the drive wheels can end "
            "up carrying no load and the robot will not move" % clearance
        )
        # Large enough to beat contact-solver noise, small enough that the pitch
        # onto a caster stays negligible over the 0.54 m wheelbase.
        assert 0.001 <= clearance <= 0.010

    def test_sensors_have_non_zero_noise(self):
        """A noiseless simulation produces numbers that cannot be reproduced on
        hardware."""
        text = GAZEBO_XACRO.read_text(encoding="utf-8")
        stddevs = [float(v) for v in re.findall(r"<stddev>([\d.eE+-]+)</stddev>", text)]
        assert stddevs, "no sensor noise configured at all"
        assert all(s > 0 for s in stddevs)


class TestZones:
    def test_zone_file_parses_and_is_non_empty(self):
        zones = _load_yaml(ZONES)["zones"]
        assert len(zones) >= 4

    def test_zone_names_are_unique(self):
        names = [z["name"] for z in _load_yaml(ZONES)["zones"]]
        assert len(names) == len(set(names))

    def test_zone_height_bands_are_valid(self):
        for zone in _load_yaml(ZONES)["zones"]:
            assert zone["z_max"] > zone["z_min"]

    def test_pick_faces_sit_below_the_first_shelf_beam(self):
        """A zone reaching above 0.80 m contains the beam and its stock, and is
        then permanently occupied."""
        for zone in _load_yaml(ZONES)["zones"]:
            if zone["name"].startswith("pick_face"):
                assert zone["z_min"] >= 0.20, "%s reaches floor clutter" % zone["name"]

    def test_zones_lie_inside_the_mapped_area(self):
        map_meta = _load_yaml(MAP_YAML)
        origin_x, origin_y = map_meta["origin"][0], map_meta["origin"][1]
        for zone in _load_yaml(ZONES)["zones"]:
            for x, y in zone["polygon"]:
                assert origin_x < x < origin_x + 32.0, "%s falls outside the map" % zone["name"]
                assert origin_y < y < origin_y + 14.0, "%s falls outside the map" % zone["name"]


class TestBehaviourTrees:
    def test_trees_are_well_formed_xml(self):
        for path in (SRC / "amr_navigation" / "behavior_trees").glob("*.xml"):
            ET.parse(path)

    def test_params_file_contains_no_launch_substitutions(self):
        """A Nav2 params file is plain YAML — it resolves nothing.

        Regression guard. `$(find-pkg-share amr_navigation)/behavior_trees/...`
        in nav2_params.yaml is the natural thing to write and is silently
        wrong: rclcpp passes the string through verbatim, bt_navigator dies
        with `Couldn't open input XML file: $`, the lifecycle manager aborts
        the entire navigation bringup, and every goal is then rejected in under
        a second with nothing in the mission log explaining why.

        Paths that need resolving belong in the launch file, which can do it.
        """
        text = NAV2_PARAMS.read_text(encoding="utf-8")
        for line in text.splitlines():
            code = line.split("#", 1)[0]  # the explanation in the comments is fine
            assert "$(find-pkg-share" not in code and "$(find " not in code, (
                "nav2_params.yaml contains an unexpanded launch substitution, "
                "which reaches the node as a literal string: %r" % line.strip()
            )

    def test_launch_file_supplies_the_behaviour_tree_paths(self):
        """Since the YAML cannot, the launch file must — and to real files."""
        launch = (SRC / "amr_navigation" / "launch" / "navigation.launch.py").read_text(
            encoding="utf-8"
        )
        trees = SRC / "amr_navigation" / "behavior_trees"
        for key in ("default_nav_to_pose_bt_xml", "default_nav_through_poses_bt_xml"):
            assert key in launch, "%s is set nowhere; bt_navigator will use the Nav2 default" % key

        for name in ("navigate_w_recovery.xml", "navigate_through_poses_w_recovery.xml"):
            assert name in launch, "%s is not referenced by the launch file" % name
            assert (trees / name).exists(), "%s is referenced but does not exist" % name

    def test_evaluation_launch_shuts_down_with_the_mission(self):
        """Otherwise pose_logger spins forever and every run costs a timeout."""
        launch = (SRC / "amr_bringup" / "launch" / "evaluation.launch.py").read_text(
            encoding="utf-8"
        )
        assert "OnProcessExit" in launch and "Shutdown" in launch, (
            "evaluation.launch.py does not shut down when the mission ends, so a "
            "run only finishes when the calling script's timeout fires"
        )

    def test_recovery_node_names_match_what_the_mission_runner_watches(self):
        """The mission runner identifies recoveries by BT node name. Renaming a
        node in the XML without updating the runner silently produces an empty
        recovery matrix."""
        import sys

        sys.path.insert(0, str(SRC / "amr_bringup"))
        text = (SRC / "amr_navigation" / "behavior_trees" / "navigate_w_recovery.xml").read_text(
            encoding="utf-8"
        )
        # Names the runner looks for, kept in sync by this assertion.
        for name in ("ClearLocalCostmap-Recovery", "ClearGlobalCostmap-Recovery"):
            assert name in text, "%s is watched by mission_runner but absent from the tree" % name
        for element in ("<Spin", "<BackUp", "<Wait"):
            assert element in text, "%s recovery missing from the tree" % element

    def test_recovery_ladder_is_ordered_cheapest_first(self):
        """Costmap clears before motion; motion before waiting."""
        text = (SRC / "amr_navigation" / "behavior_trees" / "navigate_w_recovery.xml").read_text(
            encoding="utf-8"
        )
        round_robin = text.split('<RoundRobin name="RecoveryActions">')[1].split("</RoundRobin>")[0]
        order = [
            round_robin.index("ClearLocalCostmap-Recovery"),
            round_robin.index("ClearGlobalCostmap-Recovery"),
            round_robin.index("<Spin"),
            round_robin.index("<BackUp"),
            round_robin.index("<Wait"),
        ]
        assert order == sorted(order), "recovery actions are not in escalating order"


class TestMissionWaypoints:
    """The mission has to be executable in a 2.8 m aisle.

    Nothing in Nav2 checks that a goal is sensible for the space it is in, so
    these assertions stand in for the judgement a person would apply.
    """

    @staticmethod
    def _waypoints():
        return _load_yaml(WAYPOINTS)["waypoints"]

    def test_waypoints_stay_on_the_aisle_centreline(self):
        """Racking faces sit at |y| = 1.35 and the robot is 0.55 m wide."""
        for w in self._waypoints():
            assert abs(float(w["y"])) < 0.5, (
                "waypoint at y=%.2f is off the centreline in a 2.8 m aisle" % w["y"]
            )

    def test_waypoint_headings_match_the_approach_direction(self):
        """A heading that differs from the approach makes the robot rotate
        while it is still translating.

        DWB's path critics reward aligning with the planned path, so a waypoint
        asking for a heading the robot is not already on turns it mid-leg. In a
        2.8 m aisle that sweeps a 0.70 m body into the racking, and it is
        exactly what happened: the robot held the centreline to within 0.16 m
        for 19 m, then on the leg toward a waypoint asking for yaw = pi it began
        turning at x = 10.2 and wedged against the rack face at y = 1.24 facing
        across the aisle, with no recovery having fired.

        A direction reversal is legitimate, but it must happen where the robot
        is already stopped, so DWB can execute it as an in-place rotation.
        """
        import math

        waypoints = self._waypoints()
        previous_x = -12.0  # spawn, from simulation/generate_world.py
        for index, w in enumerate(waypoints):
            dx = float(w["x"]) - previous_x
            if abs(dx) > 0.5:  # a leg with real travel defines an approach
                approach = 0.0 if dx > 0 else math.pi
                diff = abs((float(w["yaw"]) - approach + math.pi) % (2 * math.pi) - math.pi)
                assert diff < 0.1, (
                    "waypoint %d asks for yaw %.3f but is approached on heading %.3f; "
                    "the robot would rotate while translating"
                    % (index, float(w["yaw"]), approach)
                )
            previous_x = float(w["x"])

    def test_waypoints_clear_the_aisle_pallets(self):
        """The world leaves pallets at x in [-7.1, -5.9] and [3.4, 4.6]."""
        for w in self._waypoints():
            for x0, x1 in ((-7.1, -5.9), (3.4, 4.6)):
                assert not (x0 - 0.6) <= float(w["x"]) <= (x1 + 0.6), (
                    "waypoint at x=%.2f sits on the pallet spanning [%.1f, %.1f]"
                    % (w["x"], x0, x1)
                )


class TestGzSim:
    """The Gazebo Classic to gz-sim migration's own failure modes.

    Each of these is silent at runtime: a topic that is not bridged simply
    never arrives, a system that is not loaded simply does nothing, and a
    static model moved by the fault injector stays where it was as far as the
    sensors are concerned.
    """

    @staticmethod
    def _bridge():
        return {entry["gz_topic_name"]: entry for entry in _load_yaml(GZ_BRIDGE)}

    def test_every_gz_topic_the_stack_needs_is_bridged(self):
        bridge = self._bridge()
        for gz_topic in ("/clock", "/odom_raw", "/imu/data", "/lidar_3d/points", "/joint_states",
                         "/ground_truth/pose"):
            assert gz_topic in bridge, "%s is not bridged into ROS" % gz_topic
            assert bridge[gz_topic]["direction"] == "GZ_TO_ROS"
        assert bridge["/cmd_vel"]["direction"] == "ROS_TO_GZ"

    def test_sensor_topics_in_the_urdf_match_the_bridge(self):
        text = GAZEBO_XACRO.read_text(encoding="utf-8")
        topics = set(re.findall(r"<topic>([^<]+)</topic>", text))
        bridge = self._bridge()
        assert {"/imu/data", "/joint_states", "/cmd_vel", "/lidar_3d"} <= topics
        assert "/lidar_3d/points" in bridge
        # The 2D LiDAR topic is the scan_topic xacro argument, bridged by the
        # launch file because it moves when the dropout relay is in the path.
        assert "/$(arg scan_topic)" in topics
        launch = (SRC / "amr_bringup" / "launch" / "simulation.launch.py").read_text(encoding="utf-8")
        assert "@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan" in launch

    def test_every_sensor_names_its_tf_frame(self):
        """Without gz_frame_id the message carries a scoped gz name that is not in TF."""
        text = GAZEBO_XACRO.read_text(encoding="utf-8")
        sensors = re.findall(r'<gazebo reference="([^"]+)">\s*<sensor', text)
        assert sorted(sensors) == ["imu_link", "lidar_2d_link", "lidar_3d_link"]
        for link in sensors:
            block = text.split('<gazebo reference="%s">' % link)[1].split("</gazebo>")[0]
            assert "<gz_frame_id>%s</gz_frame_id>" % link in block

    def test_world_loads_the_systems_the_stack_relies_on(self):
        world = WORLD.read_text(encoding="utf-8")
        for system in ("Physics", "UserCommands", "SceneBroadcaster", "Sensors", "Imu"):
            assert "gz::sim::systems::%s" % system in world, "world does not load %s" % system
        assert "<render_engine>ogre2</render_engine>" in world

    def test_world_fetches_nothing_from_the_network(self):
        """model:// includes resolve through Fuel in gz-sim; CI must not depend on that."""
        world = WORLD.read_text(encoding="utf-8")
        assert "<include>" not in world and "model://" not in world
        assert "gazebo.material" not in world, "Classic material scripts do not exist in gz-sim"

    def test_world_keeps_the_classic_physics_step(self):
        world = WORLD.read_text(encoding="utf-8")
        assert "<max_step_size>0.001</max_step_size>" in world

    def test_faulted_models_exist_and_are_dynamic(self):
        """gz-sim does not move a static model's collision or visual on set_pose.

        Measured: a static pallet teleported 2.35 m ahead of the robot reported
        its new pose to `gz model -p` while the forward LiDAR beams still read
        16 m, so the robot would have driven through it. Every model a fault
        moves must therefore be dynamic.
        """
        world = WORLD.read_text(encoding="utf-8")
        schedules = [SRC / "amr_bringup" / "config" / name
                     for name in ("fault_schedule.yaml", "integration_fault.yaml")]
        moved = {f["model"] for path in schedules for f in _load_yaml(path)["faults"] if "model" in f}
        assert moved - {"warehouse_amr"}, "no fault moves a prop; the check would be vacuous"
        for model in moved - {"warehouse_amr"}:
            match = re.search(r'<model name="%s">\s*<static>(\w+)</static>' % re.escape(model), world)
            assert match, "%s is moved by a fault but is not in the world" % model
            assert match.group(1) == "false", "%s is static; set_pose would not move it" % model

    def test_wheel_contact_is_a_thin_disc(self):
        """DART puts a cylinder's floor contacts at its rim edges.

        With the 50 mm tyre as the collision, the inner edges carried the load
        and the effective track was 0.45 m against DiffDrive's 0.50 m: one
        commanded turn in place came out as 397 degrees by ground truth and 358
        by wheel odometry. The contact must be thin enough that the track error
        stays under 1%.
        """
        width = _xacro_property("wheel_contact_width")
        separation = _xacro_property("wheel_separation")
        assert width < _xacro_property("wheel_width")
        assert width / separation < 0.01
        assert 'length="${wheel_contact_width}"' in URDF.read_text(encoding="utf-8")

    def test_launch_bridges_the_set_pose_service(self):
        launch = (SRC / "amr_bringup" / "launch" / "simulation.launch.py").read_text(encoding="utf-8")
        injector = (SRC / "amr_bringup" / "amr_bringup" / "nodes" / "fault_injector.py").read_text(
            encoding="utf-8")
        assert "/world/warehouse/set_pose@ros_gz_interfaces/srv/SetEntityPose" in launch
        assert '"/world/warehouse/set_pose"' in injector


class TestJazzyNav2:
    """Humble configuration that Jazzy rejects, sometimes without saying so."""

    def test_no_builtin_bt_plugins_are_listed(self, nav2):
        """Jazzy loads the built-in BT nodes itself; listing them registers each twice."""
        assert "plugin_lib_names" not in nav2["bt_navigator"]["ros__parameters"]

    def test_plugin_names_use_the_scoped_form(self):
        for line in NAV2_PARAMS.read_text(encoding="utf-8").splitlines():
            code = line.split("#", 1)[0]
            if "plugin:" in code:
                value = code.split("plugin:", 1)[1].strip().strip('"')
                assert "/" not in value, "Humble-style plugin name: %s" % value

    def test_progress_checker_uses_the_plural_key(self, nav2):
        params = nav2["controller_server"]["ros__parameters"]
        assert "progress_checker_plugin" not in params
        assert params["progress_checker_plugins"] == ["progress_checker"]

    def test_behaviour_trees_declare_btcpp_v4(self):
        for path in (SRC / "amr_navigation" / "behavior_trees").glob("*.xml"):
            root = ET.parse(path).getroot()
            assert root.get("BTCPP_format") == "4", "%s is not declared as BT.CPP v4" % path.name

    def test_rotate_to_goal_tolerance_is_set_where_dwb_reads_it(self, dwb):
        """RotateToGoal reads FollowPath.xy_goal_tolerance, not RotateToGoal.xy_goal_tolerance.

        The misnamed key was silently ignored, so the critic ran on its 0.25 m
        default against a 0.20 m goal checker. In the annulus between them DWB
        rejects every forward trajectory and the goal checker never succeeds:
        measured on gz-sim, the robot stopped 0.24 m short of every goal.
        """
        for key in ("RotateToGoal.xy_goal_tolerance", "RotateToGoal.trans_stopped_velocity"):
            assert key not in dwb, "%s is not a parameter DWB reads; set it on FollowPath" % key
        assert "xy_goal_tolerance" in dwb and "trans_stopped_velocity" in dwb

    def test_translation_ends_inside_the_goal_checker_radius(self, nav2, dwb):
        checker = nav2["controller_server"]["ros__parameters"]["general_goal_checker"]
        assert dwb["xy_goal_tolerance"] < checker["xy_goal_tolerance"], (
            "RotateToGoal stops translation at %.2f m but the goal checker needs %.2f m: "
            "a robot can stop in between and never finish" % (dwb["xy_goal_tolerance"],
                                                              checker["xy_goal_tolerance"])
        )
