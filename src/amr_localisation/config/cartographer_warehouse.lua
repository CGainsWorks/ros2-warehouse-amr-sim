-- Cartographer 2D SLAM for the warehouse mapping run.
--
-- Used once, offline, to build the occupancy grid that AMCL then localises
-- against. It is not run during navigation: Cartographer's local SLAM would
-- keep adjusting the map underneath Nav2's global costmap, and a costmap whose
-- static layer moves is a costmap whose plans are not reproducible.
--
-- The dominant difficulty in this environment is that racking bays repeat at a
-- fixed 2.7 m pitch down a 30 m aisle. Loop closure will confidently match bay
-- n against bay n+1 and fold the map in on itself, producing a warehouse that
-- is a whole bay shorter than the real one. Most of the non-default values
-- below exist to make that specific failure harder.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",

  -- Cartographer publishes map -> odom and lets the EKF keep odom -> base_link,
  -- exactly as AMCL does during navigation. Setting published_frame to
  -- base_footprint here instead would have Cartographer take over the whole
  -- chain and fight the EKF.
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,

  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,

  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,

  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,

  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- Single-threaded would work but mapping a 30 m aisle takes long enough that
-- the constraint search becomes the bottleneck.
MAP_BUILDER.num_background_threads = 4

-- ---------------------------------------------------------------------------
-- Local SLAM
-- ---------------------------------------------------------------------------

-- The IMU is used. Cartographer's 2D IMU integration constrains yaw between
-- scans, which is the one thing that most reduces the chance of a scan matching
-- into the wrong bay during a fast turn at an aisle end.
TRAJECTORY_BUILDER_2D.use_imu_data = true

-- Below 0.3 m the returns are off the robot's own body and the fork carriage.
TRAJECTORY_BUILDER_2D.min_range = 0.30
-- Well inside the sensor's 25 m maximum. Long returns down the aisle contribute
-- little to the local scan match and, being highly sensitive to small yaw
-- errors, actively degrade it.
TRAJECTORY_BUILDER_2D.max_range = 20.0
-- Where an out-of-range beam is treated as free space out to. Slightly beyond
-- max_range so the free-space carving reaches the far wall.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 21.0

-- Correlative scan matching ON. It is expensive, and for a well-behaved
-- environment the Ceres matcher alone is enough. This is not a well-behaved
-- environment: the exhaustive search over a bounded window is what stops the
-- Ceres matcher sliding into the adjacent bay's local minimum.
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
-- Half the 2.7 m bay pitch would be 1.35 m; 0.15 m is deliberately far smaller.
-- The window only has to cover the motion between scans plus odometry error, and
-- a window that reaches the next bay would let the search find it.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- Ceres weights: how much the optimiser trusts the scan match against how much
-- it trusts the odometry prior. Raising the translation weight relative to
-- occupied_space, which is the usual first instinct when a map comes out
-- smeared, is wrong here — it makes the result track odometry through a genuine
-- slip. These sit near library defaults with the rotation weight raised, since
-- yaw is the axis this environment punishes.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 60.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20

-- Motion filter: drop scans that represent almost no movement. Without this a
-- stationary robot inserts hundreds of near-identical scans into one submap and
-- makes it artificially confident.
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 5.
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.10
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.0)

-- 90 scans per submap at 10 Hz and 0.6 m/s is roughly a 5 m stretch of aisle,
-- so a submap spans about two racking bays. Larger submaps drift internally
-- before they are closed; much smaller ones give the pose graph too little
-- structure per node to match against.
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05

-- Probability applied to a cell on hit and miss. The defaults are tuned for
-- environments with more dynamic content; the warehouse map is being built from
-- a deliberate, obstacle-free mapping run, so a slightly higher hit probability
-- makes the racking uprights solid rather than speckled.
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.58
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.49

-- ---------------------------------------------------------------------------
-- Pose graph / global SLAM
-- ---------------------------------------------------------------------------

POSE_GRAPH.optimize_every_n_nodes = 90

-- The bay-aliasing defences live here.
--
-- min_score is raised well above the 0.55 default. A constraint is only
-- accepted if the submap match is genuinely strong, because a match against the
-- wrong bay in a repetitive aisle scores respectably. The cost of a high
-- threshold is fewer loop closures and therefore more residual drift; the cost
-- of a low one is a map folded a whole bay short, which is not recoverable.
POSE_GRAPH.constraint_builder.min_score = 0.68
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.72

-- Only 30% of nodes are considered for global constraints. Reduces both compute
-- and the number of chances to form a wrong constraint.
POSE_GRAPH.constraint_builder.sampling_ratio = 0.30

-- Search windows for global constraints. 7 m is enough to close a loop at the
-- end of the aisle without reaching across to a similar-looking bay two along.
POSE_GRAPH.constraint_builder.max_constraint_distance = 12.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 7.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(25.)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 7

-- Huber loss on the optimisation. A larger scale makes the optimiser less
-- willing to discount a constraint as an outlier. Kept at the default 1e1: with
-- min_score already high, wrong constraints should be rare, and being tolerant
-- of the ones that survive is worse than being robust to them.
POSE_GRAPH.optimization_problem.huber_scale = 1e1
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5

-- Run a full global optimisation at the end of the mapping run before the map
-- is written out.
POSE_GRAPH.max_num_final_iterations = 200

return options
