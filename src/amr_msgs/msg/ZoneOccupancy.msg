# Occupancy state of one named work zone.

string name

# Debounced state. This is the field to gate a robot on: it has passed the
# enter/exit hysteresis in the zone publisher.
bool occupied

# Undebounced state for this scan alone. Exposed so that the debouncing can be
# evaluated offline against the raw measurement, and so a consumer that wants a
# faster, noisier signal can have one knowingly.
bool occupied_raw

# Non-ground returns inside the zone prism this scan.
uint32 point_count

# Distance from the zone centroid to the nearest cluster centroid, in metres.
# NaN when no clusters were detected at all.
float32 nearest_cluster_distance

# DetectedObject ids intersecting this zone, indexing the DetectedObjectArray
# published on the same stamp.
int32[] cluster_ids
