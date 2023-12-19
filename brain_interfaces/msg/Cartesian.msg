# list of poses to be executed by the /compute_cartesian_motion service
geometry_msgs/Pose[] poses

# the point in space at which we start drawing the letter. 
# We only use the y and z coordinates, so make the x coordinate whatever you want
geometry_msgs/Point start_point