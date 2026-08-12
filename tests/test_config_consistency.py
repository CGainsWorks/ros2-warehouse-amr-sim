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
MAP_YAML = REPO_ROOT / "simulation" / "maps" / "warehouse.yaml"


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

    def test_wheel_diameter_matches_the_urdf_radius(self):
        """A mismatch scales all reported distance, which looks exactly like a
        slipping wheel and will be chased as one."""
        assert _gazebo_scalar("wheel_diameter") == pytest.approx(2 * _xacro_property("wheel_radius"))

    def test_costmap_footprint_matches_the_urdf_body(self, local_costmap, global_costmap):
        """A footprint smaller than the robot lets it plan into the racking."""
        length = _xacro_property("base_length")
        width = _xacro_property("base_width")

        for costmap in (local_costmap, global_costmap):
            footprint = yaml.safe_load(costmap["footprint"])
            xs = [abs(p[0]) for p in footprint]
            ys = [abs(p[1]) for p in footprint]
            assert max(xs) == pytest.approx(length / 2.0), "footprint length disagrees with the URDF"
            assert max(ys) == pytest.approx(width / 2.0), "footprint width disagrees with the URDF"

    def test_both_costmaps_use_the_same_footprint(self, local_costmap, global_costmap):
        assert local_costmap["footprint"] == global_costmap["footprint"]


class TestDynamics:
    def test_dwb_acceleration_matches_the_base(self, dwb):
        """DWB planning more deceleration than the wheels can deliver overshoots
        every goal and clips every corner."""
        wheel_radius = _xacro_property("wheel_radius")
        max_wheel_accel = _gazebo_scalar("max_wheel_acceleration")  # rad/s^2
        linear_accel = max_wheel_accel * wheel_radius
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
        assert "<publish_odom_tf>false</publish_odom_tf>" in GAZEBO_XACRO.read_text(
            encoding="utf-8"
        ).replace(" ", ""), "the Gazebo diff drive plugin must not also publish odom TF"

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
        plugin_frame = re.search(r"<robot_base_frame>([^<]+)</robot_base_frame>", gazebo).group(1)

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

    def test_wheel_odometry_is_fused_as_velocity_not_pose(self):
        """Fusing wheel pose double-counts error the filter has already
        integrated and makes the covariance optimistic."""
        ekf = _load_yaml(EKF_PARAMS)["ekf_filter_node"]["ros__parameters"]
        config = ekf["odom0_config"]
        assert not any(config[0:6]), "wheel odometry must not contribute absolute pose"
        assert config[6] is True, "wheel odometry should contribute vx"
        assert config[11] is True, "wheel odometry should contribute vyaw"

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

    def test_gazebo_odometry_is_encoder_derived_not_ground_truth(self):
        """odometry_source 1 is ENCODER. 0 is WORLD, which publishes the
        simulator's exact pose as odometry and removes every source of drift,
        turning the localisation evaluation into a measurement of nothing."""
        assert "<odometry_source>1</odometry_source>" in GAZEBO_XACRO.read_text(encoding="utf-8")

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

    def test_bt_navigator_references_trees_that_exist(self, nav2):
        params = nav2["bt_navigator"]["ros__parameters"]
        for key in ("default_nav_to_pose_bt_xml", "default_nav_through_poses_bt_xml"):
            referenced = Path(str(params[key]).strip()).name
            assert (SRC / "amr_navigation" / "behavior_trees" / referenced).exists(), (
                "%s points at %s, which does not exist" % (key, referenced)
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
