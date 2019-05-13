# Each trajectory point specifies cartesian pose,
# [, velocity]
# [, acceleration]
# [, blending radius]
# for the trajectory to be executed.

geometry_msgs/Pose pose
float64 velocity
float64 acceleration
float64 blending_radius
duration time_from_start
