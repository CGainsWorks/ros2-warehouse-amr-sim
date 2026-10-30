#!/usr/bin/env python3
"""Generate the Gazebo warehouse world.

The world is generated rather than hand-written because the same geometry has to
appear in three places and they must not drift apart:

* ``simulation/worlds/warehouse.world`` — what Gazebo simulates.
* ``amr_perception.core.synthetic.warehouse_scene`` — the ray-traced scene the
  segmentation metrics are measured on.
* ``simulation/zones/warehouse_zones.yaml`` — the pick faces the zone publisher
  watches.

A hand-maintained SDF gets edited, the synthetic scene does not, and then the
offline segmentation numbers quietly stop describing the simulated warehouse.
Deriving all three from the constants in this file makes that impossible.

Run ``make world`` (or this script directly) after changing any dimension. The
generated files are committed, so a plain checkout can run the demo without
having Python set up first.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import List, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# The single source of truth for warehouse geometry.
# ---------------------------------------------------------------------------
LENGTH = 30.0
WIDTH = 12.0
WALL_HEIGHT = 6.0
WALL_THICKNESS = 0.2

AISLE_HALF_WIDTH = 1.4  # 2.8 m clear aisle for a 0.70 x 0.50 m robot
RACK_DEPTH = 0.9
BAY_PITCH = 2.7
N_BAYS = 10
UPRIGHT_SIZE = 0.10
RACK_HEIGHT = 2.2
BEAM_LEVELS = (0.80, 1.60)
BEAM_THICKNESS = 0.08

FLOOR_PALLETS: List[Tuple[float, float]] = [(4.0, 0.35), (-6.5, -0.40)]
PEOPLE: List[Tuple[float, float]] = [(8.0, 0.0), (-3.0, 1.0), (14.0, -0.8)]

ROBOT_SPAWN = (-12.0, 0.0, 0.0)

# A loaded pallet the fault injector can move. It is parked outside the
# building, behind the 6 m walls and off the reference map, so on a normal run
# no sensor can see it and it is not part of any scene.
#
# Why a separate prop rather than teleporting the aisle pallets as Gazebo
# Classic did: in gz-sim, set_pose on a *static* model updates the entity's
# pose component but neither the physics engine nor the rendering scene follows
# it, so a teleported static pallet is invisible to both LiDARs and the robot
# drives through it. Measured: the stack moved to 2.35 m ahead of the robot,
# `gz model -p` reported the new pose, and the forward LiDAR beams still read
# 16 m. A dynamic model is moved properly, so the prop is dynamic and heavy
# enough (250 kg) that the robot cannot shove it aside.
FAULT_PALLET_SIZE = (1.2, 0.8, 0.95)
FAULT_PALLET_PARK = (0.0, 25.0)
FAULT_PALLET_MASS = 250.0

HALF_L = LENGTH / 2.0
HALF_W = WIDTH / 2.0


def bay_x_positions() -> List[float]:
    out = []
    for bay in range(N_BAYS + 1):
        x = -HALF_L + 1.5 + bay * BAY_PITCH
        if x > HALF_L - 1.0:
            break
        out.append(x)
    return out


# Visual colours, RGBA. Gazebo Classic named its materials from a script file
# (``Gazebo/Grey``); gz-sim has no material scripts, so each name maps to the
# ambient/diffuse colour it used to stand for. Purely cosmetic: neither the
# LiDARs nor the physics read a colour.
COLOURS = {
    "grey": "0.70 0.70 0.70 1",
    "dark_grey": "0.18 0.18 0.18 1",
    "orange": "1.00 0.51 0.05 1",
    "wood": "0.55 0.38 0.22 1",
    "wood_pallet": "0.65 0.50 0.32 1",
    "red": "0.80 0.10 0.10 1",
    "yellow": "0.95 0.80 0.10 1",
}


def _material(colour: str) -> str:
    rgba = COLOURS[colour]
    return (f"<material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse>"
            f"<specular>0.1 0.1 0.1 1</specular></material>")


def _box_model(name: str, x: float, y: float, z: float, sx: float, sy: float, sz: float,
               colour: str = "grey", static: bool = True) -> str:
    """A static box. Collision and visual share one geometry."""
    return f"""
    <model name="{name}">
      <static>{'true' if static else 'false'}</static>
      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
          {_material(colour)}
        </visual>
      </link>
    </model>"""


def _cylinder_model(name: str, x: float, y: float, z: float, radius: float, length: float,
                    colour: str = "grey") -> str:
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><cylinder><radius>{radius:.4f}</radius><length>{length:.4f}</length></cylinder></geometry>
        </collision>
        <visual name="visual">
          <geometry><cylinder><radius>{radius:.4f}</radius><length>{length:.4f}</length></cylinder></geometry>
          {_material(colour)}
        </visual>
      </link>
    </model>"""


def _dynamic_box_model(name: str, x: float, y: float, sx: float, sy: float, sz: float,
                       mass: float, colour: str) -> str:
    """A free box resting on the floor, with the inertia of a uniform solid."""
    ixx = mass * (sy * sy + sz * sz) / 12.0
    iyy = mass * (sx * sx + sz * sz) / 12.0
    izz = mass * (sx * sx + sy * sy) / 12.0
    return f"""
    <model name="{name}">
      <static>false</static>
      <pose>{x:.4f} {y:.4f} {sz / 2:.4f} 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>{mass:.1f}</mass>
          <inertia><ixx>{ixx:.4f}</ixx><iyy>{iyy:.4f}</iyy><izz>{izz:.4f}</izz>
            <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
        <collision name="collision">
          <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
          {_material(colour)}
        </visual>
      </link>
    </model>"""


def build_world() -> str:
    parts: List[str] = []

    # Perimeter walls.
    parts.append(_box_model("wall_west", -HALF_L - WALL_THICKNESS / 2, 0.0, WALL_HEIGHT / 2,
                            WALL_THICKNESS, WIDTH + 2 * WALL_THICKNESS, WALL_HEIGHT))
    parts.append(_box_model("wall_east", HALF_L + WALL_THICKNESS / 2, 0.0, WALL_HEIGHT / 2,
                            WALL_THICKNESS, WIDTH + 2 * WALL_THICKNESS, WALL_HEIGHT))
    parts.append(_box_model("wall_south", 0.0, -HALF_W - WALL_THICKNESS / 2, WALL_HEIGHT / 2,
                            LENGTH + 2 * WALL_THICKNESS, WALL_THICKNESS, WALL_HEIGHT))
    parts.append(_box_model("wall_north", 0.0, HALF_W + WALL_THICKNESS / 2, WALL_HEIGHT / 2,
                            LENGTH + 2 * WALL_THICKNESS, WALL_THICKNESS, WALL_HEIGHT))

    # Racking.
    for side_index, side in enumerate((-1.0, 1.0)):
        tag = "s" if side < 0 else "n"
        y_front = side * AISLE_HALF_WIDTH
        y_back = side * (AISLE_HALF_WIDTH + RACK_DEPTH)

        for bay_index, x in enumerate(bay_x_positions()):
            for depth_index, y in enumerate((y_front, y_back)):
                parts.append(_box_model(
                    f"upright_{tag}_{bay_index}_{depth_index}",
                    x, y, RACK_HEIGHT / 2,
                    UPRIGHT_SIZE, UPRIGHT_SIZE, RACK_HEIGHT,
                    colour="dark_grey",
                ))

        xs = bay_x_positions()
        beam_x0 = xs[0] - UPRIGHT_SIZE / 2
        beam_x1 = -HALF_L + 1.5 + N_BAYS * BAY_PITCH
        beam_len = beam_x1 - beam_x0
        beam_cx = (beam_x0 + beam_x1) / 2.0
        for level_index, beam_z in enumerate(BEAM_LEVELS):
            parts.append(_box_model(
                f"beam_{tag}_{level_index}",
                beam_cx, (y_front + y_back) / 2.0, beam_z + BEAM_THICKNESS / 2,
                beam_len, RACK_DEPTH, BEAM_THICKNESS,
                colour="orange",
            ))

        # Stock. Every third bay is left empty so the sensor can see through the
        # racking to the far wall; a fully stocked run hides the shelf-top-as-
        # floor failure mode that the ground removal is built to survive.
        for bay_index in range(N_BAYS):
            if bay_index % 3 == 2:
                continue
            x = -HALF_L + 1.5 + bay_index * BAY_PITCH + BAY_PITCH / 2.0
            if x > HALF_L - 1.0:
                break
            for level_index, level_z in enumerate((BEAM_LEVELS[0] + 0.08, BEAM_LEVELS[1] + 0.08)):
                parts.append(_box_model(
                    f"pallet_{tag}_{bay_index}_{level_index}",
                    x, (y_front + y_back) / 2.0, level_z + 0.075,
                    1.2, RACK_DEPTH - 0.1, 0.15, colour="wood",
                ))
                parts.append(_box_model(
                    f"carton_{tag}_{bay_index}_{level_index}",
                    x, (y_front + y_back) / 2.0, level_z + 0.15 + 0.30,
                    1.0, RACK_DEPTH - 0.2, 0.60, colour="wood_pallet",
                ))

    # Roof support columns.
    for index, x in enumerate((-HALF_L / 2.0, HALF_L / 2.0)):
        parts.append(_cylinder_model(f"column_{index}", x, HALF_W - 0.8, WALL_HEIGHT / 2,
                                     0.15, WALL_HEIGHT, colour="dark_grey"))

    # Obstacles left in the aisle, which is what Nav2 has to plan around.
    for index, (px, py) in enumerate(FLOOR_PALLETS):
        parts.append(_box_model(f"aisle_pallet_{index}", px, py, 0.075, 1.2, 0.8, 0.15,
                                colour="wood"))
        parts.append(_box_model(f"aisle_stack_{index}", px, py, 0.15 + 0.40, 1.0, 0.6, 0.80,
                                colour="wood_pallet"))

    # People, as static cylinders. Static because the dynamic-obstacle behaviour
    # exercised in the recovery runs is injected by amr_bringup's fault_injector node,
    # which teleports these models on a schedule the fault log records. A
    # <plugin> based walker would move unrepeatably and make the recovery matrix
    # impossible to reproduce.
    for index, (px, py) in enumerate(PEOPLE):
        parts.append(_cylinder_model(f"person_{index}", px, py, 1.75 / 2, 0.25, 1.75,
                                     colour="red"))

    # The fault injector's movable pallet, parked out of sight.
    parts.append(_dynamic_box_model("fault_pallet", FAULT_PALLET_PARK[0], FAULT_PALLET_PARK[1],
                                    *FAULT_PALLET_SIZE, mass=FAULT_PALLET_MASS, colour="yellow"))

    models = "\n".join(parts)

    return f"""<?xml version="1.0" ?>
<!--
  GENERATED FILE. Do not edit by hand.
  Produced by simulation/generate_world.py; run `make world` after changing it.

  Geometry here is kept identical to amr_perception.core.synthetic.warehouse_scene
  so that offline segmentation metrics describe the same warehouse the simulator
  does.
-->
<sdf version="1.9">
  <world name="warehouse">

    <!-- 1 ms step, real-time target. The step is kept at the Classic value:
         a larger one lets the caster spheres sink into the floor at speed,
         which presents as the robot randomly snagging on nothing. DART is
         gz-sim's default and only fully supported physics engine. -->
    <physics name="1ms" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- gz-sim loads nothing by default once a world names any system, so
         every system the stack relies on is listed here.
           Physics          rigid-body dynamics
           UserCommands     /world/warehouse/set_pose, used by the fault injector
           SceneBroadcaster pose and scene state for the GUI and gz topic tools
           Sensors          renders both LiDARs (gpu_lidar needs ogre2)
           Imu              the IMU is not a rendering sensor and has its own
                            system -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <!-- The Classic world included the sun and ground_plane models from the
         local model database. gz-sim would fetch them from Fuel over the
         network, which a CI runner should not depend on, so both are inline
         with the Classic models' values (ground friction mu 100 / mu2 50). -->
    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <surface><friction><ode><mu>100</mu><mu2>50</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse></material>
        </visual>
      </link>
    </model>

    <scene>
      <ambient>0.5 0.5 0.5 1</ambient>
      <background>0.75 0.78 0.82 1</background>
      <shadows>false</shadows>
      <grid>false</grid>
    </scene>
{models}

  </world>
</sdf>
"""


def build_zones() -> str:
    """Zone definitions covering the pick faces and the charger bay.

    A pick-face zone is the strip of *aisle* a person stands in to work the
    face. It deliberately stops short of the rack face and does not extend into
    the bay at all.

    That distinction is the whole design. A zone drawn across the rack face
    contains its own uprights, beams and stock, all of which are permanent,
    static returns — so the zone reads occupied for ever and the signal is
    worthless. The uprights' inner faces sit at |y| = 1.35; the zones stop at
    1.25, leaving a 0.10 m margin that absorbs both the localisation error the
    zone node tolerates and the LiDAR's own range noise.

    The strip is 0.75 m deep, which comfortably contains a person standing at
    the face (0.25 m radius, so 0.75-1.25 with their back to the rack) without
    reaching the aisle centreline where the robot itself passes.
    """
    lines = [
        "# GENERATED FILE. Do not edit by hand.",
        "# Produced by simulation/generate_world.py; run `make world` after changing it.",
        "#",
        "# Coordinates are in the map frame, metres. Each zone is a floor polygon",
        "# plus a height band, forming a vertical prism.",
        "",
        "zones:",
    ]

    # Both offsets move towards the aisle centre, away from the rack.
    face_clearance = 0.15  # gap between the rack face and the zone edge
    strip_depth = 0.75  # how far the working strip reaches into the aisle

    for side, tag in ((1.0, "north"), (-1.0, "south")):
        y_face = side * AISLE_HALF_WIDTH
        y_near = y_face - side * face_clearance
        y_far = y_face - side * (face_clearance + strip_depth)

        for group, bays in enumerate(((0, 1, 2), (3, 4, 5), (6, 7, 8))):
            x0 = -HALF_L + 1.5 + bays[0] * BAY_PITCH
            x1 = -HALF_L + 1.5 + (bays[-1] + 1) * BAY_PITCH
            lo_y, hi_y = sorted((y_near, y_far))
            lines += [
                f"  - name: pick_face_{tag}_{group}",
                "    description: >-",
                f"      Pick face covering bays {bays[0]}-{bays[-1]} on the {tag} run.",
                "    polygon:",
                f"      - [{x0:.2f}, {lo_y:.2f}]",
                f"      - [{x1:.2f}, {lo_y:.2f}]",
                f"      - [{x1:.2f}, {hi_y:.2f}]",
                f"      - [{x0:.2f}, {hi_y:.2f}]",
                "    # Above floor clutter, below the first beam at 0.80 m, so a",
                "    # loaded pallet on the bottom level does not read as a person.",
                "    z_min: 0.25",
                "    z_max: 1.80",
                "    min_points: 15",
                "",
            ]

    lines += [
        "  - name: charger_bay",
        "    description: >-",
        "      Charging dock at the west end. Occupied means another robot is",
        "      already on the dock.",
        "    polygon:",
        f"      - [{-HALF_L + 0.3:.2f}, -1.20]",
        f"      - [{-HALF_L + 2.0:.2f}, -1.20]",
        f"      - [{-HALF_L + 2.0:.2f}, 1.20]",
        f"      - [{-HALF_L + 0.3:.2f}, 1.20]",
        "    z_min: 0.15",
        "    z_max: 1.80",
        "    # Larger than the pick faces: a robot on the dock presents far more",
        "    # returns than a person, and the extra margin rejects a single",
        "    # stray reflection off the dock's own charging contacts.",
        "    min_points: 40",
        "",
    ]
    return "\n".join(lines)


MAP_RESOLUTION = 0.05
MAP_ORIGIN = (-16.0, -7.0)
MAP_WIDTH_M = 32.0
MAP_HEIGHT_M = 14.0

# The 2D LiDAR plane. Only geometry spanning this height is visible to the
# sensor AMCL localises with, so only that geometry belongs in the map.
SCAN_HEIGHT = 0.35


def build_map() -> Tuple[bytes, str]:
    """Rasterise the permanent structure into an occupancy grid.

    This is a *geometric reference map*, generated from the same constants as
    the world, and it is labelled as such wherever it is used. It is not the
    output of the Cartographer run — ``make map`` produces that, and the mapping
    pipeline is the thing being demonstrated. This exists so that a fresh clone
    can run ``make demo`` without first sitting through a mapping run, and so
    that the navigation stack can be exercised against a map with no SLAM error
    in it when the point is to isolate a navigation problem from a mapping one.

    WHAT IS DELIBERATELY NOT IN IT:

    * People. They move; a static map containing them would have AMCL trying to
      match against a person who has walked away.
    * The pallets left in the aisle. Those are exactly the obstacles the
      navigation stack is supposed to detect and plan around at runtime. Baking
      them into the map would make the obstacle layer look like it was working
      when it was not.
    * Racking beams and stock above 0.35 m, which the 2D LiDAR cannot see.
    """
    nx = int(round(MAP_WIDTH_M / MAP_RESOLUTION))
    ny = int(round(MAP_HEIGHT_M / MAP_RESOLUTION))
    # 254 free, 0 occupied, 205 unknown. Nothing here is unknown: the map is
    # generated from complete geometry.
    grid = np.full((ny, nx), 254, dtype=np.uint8)

    def fill_box(lo_x, lo_y, hi_x, hi_y) -> None:
        i0 = max(0, int(math.floor((lo_x - MAP_ORIGIN[0]) / MAP_RESOLUTION)))
        i1 = min(nx, int(math.ceil((hi_x - MAP_ORIGIN[0]) / MAP_RESOLUTION)))
        j0 = max(0, int(math.floor((lo_y - MAP_ORIGIN[1]) / MAP_RESOLUTION)))
        j1 = min(ny, int(math.ceil((hi_y - MAP_ORIGIN[1]) / MAP_RESOLUTION)))
        if i1 > i0 and j1 > j0:
            grid[j0:j1, i0:i1] = 0

    # Perimeter walls.
    fill_box(-HALF_L - WALL_THICKNESS, -HALF_W - WALL_THICKNESS, -HALF_L, HALF_W + WALL_THICKNESS)
    fill_box(HALF_L, -HALF_W - WALL_THICKNESS, HALF_L + WALL_THICKNESS, HALF_W + WALL_THICKNESS)
    fill_box(-HALF_L - WALL_THICKNESS, -HALF_W - WALL_THICKNESS, HALF_L + WALL_THICKNESS, -HALF_W)
    fill_box(-HALF_L - WALL_THICKNESS, HALF_W, HALF_L + WALL_THICKNESS, HALF_W + WALL_THICKNESS)

    # Racking uprights, which span 0 to 2.2 m and so cross the scan plane.
    half = UPRIGHT_SIZE / 2.0
    for side in (-1.0, 1.0):
        for depth_offset in (0.0, RACK_DEPTH):
            y = side * (AISLE_HALF_WIDTH + depth_offset)
            for x in bay_x_positions():
                fill_box(x - half, y - half, x + half, y + half)

    # Roof support columns.
    for x in (-HALF_L / 2.0, HALF_L / 2.0):
        y = HALF_W - 0.8
        fill_box(x - 0.15, y - 0.15, x + 0.15, y + 0.15)

    # PGM is written top row first, so flip the y axis: image row 0 is the
    # highest y. Getting this wrong produces a map that is a perfect mirror of
    # the world, which localises beautifully and drives into walls.
    flipped = np.flipud(grid)
    header = ("P5\n%d %d\n255\n" % (nx, ny)).encode("ascii")

    yaml_text = (
        "# GENERATED FILE. Do not edit by hand.\n"
        "# Produced by simulation/generate_world.py; run `make world` after changing it.\n"
        "#\n"
        "# This is a geometric reference map built from the world's own constants,\n"
        "# NOT the output of the Cartographer mapping run. Run `make map` to build\n"
        "# the real one. See build_map() in generate_world.py for what is and is\n"
        "# not included and why.\n"
        "image: warehouse.pgm\n"
        "mode: trinary\n"
        "resolution: %.3f\n"
        "origin: [%.3f, %.3f, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.25\n" % (MAP_RESOLUTION, MAP_ORIGIN[0], MAP_ORIGIN[1])
    )
    return header + flipped.tobytes(), yaml_text


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--root",
        type=Path,
        default=Path(__file__).resolve().parent.parent,
        help="Repository root. Defaults to the parent of simulation/.",
    )
    args = parser.parse_args()

    world_path = args.root / "simulation" / "worlds" / "warehouse.world"
    zones_path = args.root / "simulation" / "zones" / "warehouse_zones.yaml"
    map_pgm = args.root / "simulation" / "maps" / "warehouse.pgm"
    map_yaml = args.root / "simulation" / "maps" / "warehouse.yaml"

    for path in (world_path, zones_path, map_pgm):
        path.parent.mkdir(parents=True, exist_ok=True)

    # LF explicitly. Path.write_text translates newlines to the platform default,
    # so generating on Windows would emit CRLF, and CI's "generated files are up
    # to date" check would then fail purely on line endings for anyone who
    # regenerated on the wrong platform.
    def write_text(path: Path, text: str) -> None:
        with open(path, "w", encoding="utf-8", newline="\n") as handle:
            handle.write(text)

    write_text(world_path, build_world())
    write_text(zones_path, build_zones())

    pgm_bytes, yaml_text = build_map()
    map_pgm.write_bytes(pgm_bytes)
    write_text(map_yaml, yaml_text)

    for path in (world_path, zones_path, map_pgm, map_yaml):
        print("wrote %s" % path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
