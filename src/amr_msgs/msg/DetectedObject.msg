# One connected component of the non-ground LiDAR return.
#
# This is a geometric detection, not a semantic one. Nothing in the pipeline
# classifies a cluster as a person or a pallet; downstream consumers that care
# use the dimensions and the zone it falls in. Adding a class field here that is
# always zero would be worse than not having one.

# Frame-local identifier. Reassigned every scan by descending point count, so it
# is stable within a message and meaningless across messages. There is no
# tracker in this pipeline; see docs/limitations.md.
int32 id

# Raw returns behind this cluster, before voxel downsampling.
uint32 point_count

geometry_msgs/Point centroid
geometry_msgs/Point bbox_min
geometry_msgs/Point bbox_max

# bbox_max - bbox_min, duplicated so consumers need not recompute it.
geometry_msgs/Vector3 dimensions
