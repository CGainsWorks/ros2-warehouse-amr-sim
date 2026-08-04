# ros2-warehouse-amr-sim

> **Status: planned. This repository is a brief, not an implementation.**
> Nothing here is built yet. The sections below define what goes in it.

> ### Clean-room reimplementation
> This repository is an **independent reimplementation of a problem class**, built from
> scratch on personal time using public or synthetic data and open-source simulation.
> It is **not** derived from, and contains no material belonging to, any employer or
> client: no employer source code, schematics, calibration data, client data or internal
> documentation. It exists to demonstrate the architecture and engineering approach only.

An autonomous mobile robot navigation stack for a simulated warehouse, plus a 3D LiDAR segmentation pipeline that produces occupancy state for defined work zones. Built entirely in Gazebo with public datasets, demonstrating the same architecture as an industrial AMR deployment without any employer material.

---

## What this covers

CV entries this repository is the evidence for:

- Warehouse AMR Nav2 navigation stack (Convex Technology)
- LiDAR point-cloud segmentation for workspace occupancy (Convex Technology)

---

## Requirements

### Hardware

- None required. Simulation only
- Optional physical target: differential-drive AMR with 2D LiDAR, IMU and wheel encoders
- Simulation substitute: Gazebo warehouse world with a TurtleBot or Nav2 demo robot

### Software

- ROS2 Humble
- Cartographer SLAM
- AMCL
- Nav2 with the DWB local planner
- robot_localization EKF
- Nav2 behaviour trees
- PCL and Open3D for segmentation
- SemanticKITTI or a simulated LiDAR for the segmentation dataset

### Report and documentation deliverables

- Architecture note
- Parameter rationale for costmaps and DWB
- Localisation accuracy results against Gazebo ground truth
- Recovery behaviour matrix
- Segmentation accuracy and per-scan latency

---

## Inputs

- LiDAR scans
- IMU
- Wheel odometry
- Static map
- Mission goal sequence
- Workspace boundary definition

## Outputs

- Fused odometry
- Pose estimate with covariance
- Global and local costmaps
- Planned paths
- Velocity commands
- Mission status and recovery events
- Segmented and labelled clusters
- 3D occupancy grid and per-zone occupancy booleans

---

## Intended structure

```
src/
  amr_bringup/            launch and params
  amr_description/        URDF and meshes
  amr_navigation/         Nav2 config, behaviour trees
  amr_localisation/       Cartographer and AMCL config, EKF
  amr_perception/         point-cloud segmentation, occupancy zones
simulation/               warehouse worlds and models
analysis/                 localisation error and segmentation metrics
docker/
docs/
```

Plus the standard baseline: `README.md`, `LICENSE`, `.gitignore`, `.gitattributes`
(Git LFS rules for weights, bags and media), `.github/workflows/ci.yml`, `docs/`,
`scripts/setup.sh`, `scripts/run_demo.sh` and `tests/`.

---

## Build checklist

- [ ] Gazebo warehouse world and simulated AMR
- [ ] Cartographer producing a map of the warehouse
- [ ] AMCL localising against the saved map
- [ ] EKF fusing LiDAR, IMU and odometry
- [ ] Nav2 with tuned costmaps and DWB navigating goal sequences
- [ ] Recovery behaviours exercised under induced failures
- [ ] Ground-plane removal and clustering on the LiDAR cloud
- [ ] Zone occupancy publisher
- [ ] Localisation error evaluation against Gazebo ground truth

---

## Definition of done

This repository is finished when:

- [ ] The whole thing runs from one command on a clean machine.
- [ ] The README opens with a diagram or a GIF, not a wall of text.
- [ ] Every number quoted in the README is reproducible from a script in the repo.
- [ ] Limitations are stated honestly in their own section.
- [ ] A licence is declared.
- [ ] Large binaries live in Releases or LFS, not in the Git history.
- [ ] The repository URL is added to the matching CV bullet point.

---

## Starter prompt

Paste this into your coding agent of choice to begin.

<details>
<summary>Expand</summary>

```
You are helping me build a portfolio project from scratch. Read this brief and then
propose a concrete implementation plan before writing any code.

PROJECT: ros2-warehouse-amr-sim
An autonomous mobile robot navigation stack for a simulated warehouse, plus a 3D LiDAR segmentation pipeline that produces occupancy state for defined work zones. Built entirely in Gazebo with public datasets, demonstrating the same architecture as an industrial AMR deployment without any employer material.

TARGET STACK
- ROS2 Humble
- Cartographer SLAM
- AMCL
- Nav2 with the DWB local planner
- robot_localization EKF
- Nav2 behaviour trees
- PCL and Open3D for segmentation
- SemanticKITTI or a simulated LiDAR for the segmentation dataset

HARDWARE CONTEXT
- None required. Simulation only
- Optional physical target: differential-drive AMR with 2D LiDAR, IMU and wheel encoders
- Simulation substitute: Gazebo warehouse world with a TurtleBot or Nav2 demo robot

THE SYSTEM MUST TAKE THESE INPUTS
- LiDAR scans
- IMU
- Wheel odometry
- Static map
- Mission goal sequence
- Workspace boundary definition

AND PRODUCE THESE OUTPUTS
- Fused odometry
- Pose estimate with covariance
- Global and local costmaps
- Planned paths
- Velocity commands
- Mission status and recovery events
- Segmented and labelled clusters
- 3D occupancy grid and per-zone occupancy booleans

INTENDED REPOSITORY LAYOUT
src/
  amr_bringup/            launch and params
  amr_description/        URDF and meshes
  amr_navigation/         Nav2 config, behaviour trees
  amr_localisation/       Cartographer and AMCL config, EKF
  amr_perception/         point-cloud segmentation, occupancy zones
simulation/               warehouse worlds and models
analysis/                 localisation error and segmentation metrics
docker/
docs/

CONSTRAINTS
- Everything must run from a single command: `docker compose up` or `make demo`.
- No placeholder or stub code presented as working. If something is not implemented, say so.
- Every quantitative claim in the README must be reproducible from a script in this repo.
- Use only public or synthetic data. Record the licence of every dataset used.
- This is a clean-room reimplementation. Do not reproduce any proprietary design, protocol detail or dataset. Build the equivalent capability independently.

START BY
1. Confirming or challenging the proposed repository layout.
2. Listing the specific libraries and versions you intend to pin.
3. Identifying the single riskiest technical unknown and how to de-risk it first.
Then work through the milestones in the README build checklist, one at a time.

```

</details>

---

## Notes

- Do not backdate commits to match CV dates. Publishing older work today is normal and expected.
- Record dataset licences before training on anything. Several common research datasets are
  non-commercial only.
- Flip this repository to public once there is something in it worth showing.
