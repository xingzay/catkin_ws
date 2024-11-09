include "tic_mapping.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}
POSE_GRAPH.constraint_builder.sampling_ratio = 0.1
POSE_GRAPH.global_sampling_ratio = 0.002
POSE_GRAPH.optimize_every_n_nodes = 20
return options
