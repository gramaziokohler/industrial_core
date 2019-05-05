# Each trajectory point specifies cartesian pose,
# [, linar and angular velocity]
# [, acceleration]
# [, blending radius]
# for the trajectory to be executed.

geometry_msgs/Pose pose
geometry_msgs/Twist velocity
float64 acceleration
float64 blending_radius
duration time_from_start
