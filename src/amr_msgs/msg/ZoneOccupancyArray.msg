# Occupancy of every configured work zone, published as one message.
#
# One message rather than a topic per zone: a consumer deciding whether to enter
# an aisle needs a consistent snapshot across zones, and per-zone topics give it
# a set of independently-timed booleans that can disagree.
std_msgs/Header header

ZoneOccupancy[] zones

# True when the zone states were computed from a transform that was within
# tolerance of the scan stamp. False means the pose used to place the zones was
# extrapolated or stale, and the states should be treated as unreliable rather
# than as clear.
bool transform_valid
