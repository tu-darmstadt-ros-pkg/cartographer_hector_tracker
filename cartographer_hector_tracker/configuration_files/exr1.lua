-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.


include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "world",
  tracking_frame = "base_link",
  published_frame = "odom",
  matched_pointcloud_frame = "lidar_frame",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = false,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.1,
  pose_publish_period_sec = 1e-3,
  trajectory_publish_period_sec = 1e-1,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}



--POSE_GRAPH.optimization_problem.acceleration_weight = 1e-10
--POSE_GRAPH.optimization_problem.rotation_weight = 1e-10 --3e5
--POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5
--POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5
--POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
--POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5
--POSE_GRAPH.optimization_problem.fixed_frame_pose_translation_weight = 1e1
--POSE_GRAPH.optimization_problem.fixed_frame_pose_rotation_weight = 1e2
--POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 1000

POSE_GRAPH.optimization_problem.log_solver_summary = false


--POSE_GRAPH.matcher_translation_weight = 5e2
--POSE_GRAPH.matcher_rotation_weight = 1.6e3



TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 8
TRAJECTORY_BUILDER_3D.min_range = 1.0
TRAJECTORY_BUILDER_3D.max_range = 60
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 300
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 5

TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.05
TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.05

MAP_BUILDER.use_trajectory_builder_3d = true


TRAJECTORY_BUILDER_3D.submaps.num_range_data = 10000000000
POSE_GRAPH.constraint_builder.sampling_ratio = 0.000000001
POSE_GRAPH.global_sampling_ratio = 0.000000001
POSE_GRAPH.optimize_every_n_nodes = 10000000000
POSE_GRAPH.constraint_builder.min_score = 1.93
POSE_GRAPH.log_residual_histograms = false

TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.58
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.48

TRAJECTORY_BUILDER_3D.optimizing_local_trajectory_builder.high_resolution_grid_weight = 250.
TRAJECTORY_BUILDER_3D.optimizing_local_trajectory_builder.low_resolution_grid_weight = 50.
TRAJECTORY_BUILDER_3D.optimizing_local_trajectory_builder.velocity_weight = 8e1
TRAJECTORY_BUILDER_3D.optimizing_local_trajectory_builder.translation_weight = 1e2
TRAJECTORY_BUILDER_3D.optimizing_local_trajectory_builder.rotation_weight = 5e1 --2e2
TRAJECTORY_BUILDER_3D.optimizing_local_trajectory_builder.odometry_translation_weight = 1e1
TRAJECTORY_BUILDER_3D.optimizing_local_trajectory_builder.odometry_rotation_weight = 1e1
-- TRAJECTORY_BUILDER_3D.optimizing_local_trajectory_builder.scans_per_optimization_update = 1
TRAJECTORY_BUILDER_3D.optimizing_local_trajectory_builder.initialize_map_orientation_with_imu = true
TRAJECTORY_BUILDER_3D.optimizing_local_trajectory_builder.calibrate_imu = true
TRAJECTORY_BUILDER_3D.optimizing_local_trajectory_builder.imu_integrator = "RK4"
TRAJECTORY_BUILDER_3D.optimizing_local_trajectory_builder.imu_cost_term =  "PREINTEGRATION"

TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.range_data_inserter_type = "TSDF_INSERTER_3D"
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.tsdf_range_data_inserter.relative_truncation_distance = 3.0
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.tsdf_range_data_inserter.weight_function_epsilon = 1.0
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.tsdf_range_data_inserter.num_free_space_voxels = 0
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.tsdf_range_data_inserter.normal_estimate_max_nn = 10
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.tsdf_range_data_inserter.normal_estimate_radius = 0.5
TRAJECTORY_BUILDER_3D.submaps.grid_type = "TSDF"



--POSE_GRAPH.optimization_problem.acceleration_weight = 1e-10
--POSE_GRAPH.optimization_problem.rotation_weight = 1e-10 --3e5
POSE_GRAPH.optimization_problem.fix_z_in_3d = false

TRAJECTORY_BUILDER_3D.motion_filter.max_distance_meters = 0
POSE_GRAPH.optimization_problem.use_online_imu_extrinsics_in_3d = false


TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.tsdf_range_data_inserter.relative_truncation_distance  = 2.0
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.tsdf_range_data_inserter.normal_estimate_max_nn = 20
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.tsdf_range_data_inserter.normal_estimate_radius = 2.0
TRAJECTORY_BUILDER_3D.optimizing_local_trajectory_builder.velocity_weight  = 80
TRAJECTORY_BUILDER_3D.optimizing_local_trajectory_builder.translation_weight  = 100
TRAJECTORY_BUILDER_3D.min_range  = 0.75
TRAJECTORY_BUILDER_3D.max_range  = 60
TRAJECTORY_BUILDER_3D.optimizing_local_trajectory_builder.rotation_weight  = 8
TRAJECTORY_BUILDER_3D.optimizing_local_trajectory_builder.calibrate_imu  = false
TRAJECTORY_BUILDER_3D.optimizing_local_trajectory_builder.ct_window_horizon  = 0.5
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.tsdf_range_data_inserter.project_sdf_distance_to_scan_normal  = true
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.tsdf_range_data_inserter.relative_truncation_distance  = 2.5
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.tsdf_range_data_inserter.normal_estimate_max_nn  = 30
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.tsdf_range_data_inserter.normal_estimate_radius  = 2.0
TRAJECTORY_BUILDER_3D.optimizing_local_trajectory_builder.high_resolution_grid_weight = 1250.
TRAJECTORY_BUILDER_3D.optimizing_local_trajectory_builder.low_resolution_grid_weight = 0.


TRAJECTORY_BUILDER_3D.submaps.high_resolution_max_range = 0
TRAJECTORY_BUILDER_3D.optimizing_local_trajectory_builder.initialization_duration =  0.20
TRAJECTORY_BUILDER_3D.optimizing_local_trajectory_builder.optimization_rate =  0.10
TRAJECTORY_BUILDER_3D.optimizing_local_trajectory_builder.ct_window_horizon =  0.30
TRAJECTORY_BUILDER_3D.optimizing_local_trajectory_builder.ct_window_rate =  0.10
TRAJECTORY_BUILDER_3D.optimizing_local_trajectory_builder.sync_control_points_with_range_data = true
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.tsdf_range_data_inserter.normal_computation_method = "TRIANGLE_FILL_IN"

return options