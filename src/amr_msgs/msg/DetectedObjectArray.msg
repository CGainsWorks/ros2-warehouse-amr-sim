# All clusters from one scan.
#
# header.frame_id is the frame the geometry is expressed in (base_footprint for
# the raw segmentation output, map once the zone node has transformed it), and
# header.stamp is copied from the originating PointCloud2 rather than set to
# "now", so that consumers can measure true sensor-to-output latency.
std_msgs/Header header

DetectedObject[] objects

# Wall-clock milliseconds spent inside the segmentation pipeline for this scan,
# excluding message conversion. Published rather than logged because the node's
# usefulness depends on it staying under the scan period, and a number you have
# to grep the logs for is a number nobody checks.
float32 processing_ms

# Ground plane actually used, as normal (x, y, z) and offset d, satisfying
# n . p + d = 0. All zero if the plane fit failed and every point was therefore
# treated as an obstacle.
float32[4] ground_plane
