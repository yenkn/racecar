define laser ranger
(
  name "laser_link"
  sensor
  (
    samples 360
    range_min 0.0
    range_max 8.0
    fov 360.0
  )
  # generic model properties
  color "black"
  size [ 0.06 0.15 0.03 ]
)

#
# Robot model:
# footprint (counter-clockwise): [-0.1,-0.125], [0.5,-0.125], [0.5,0.125], [-0.1,0.125]
# center of rotation: [0,0]
# wheelbase: 0.4

define carlike_robot position
(
  name "true_odom"
  pose [ 0.0 0.0 0.0 0.0 ]

  odom_error [0.3 0.1 0.0 0.1]

  size [ 0.44 0.20 0.27 ] # this models the footprint (rectangular), but shifted such that the bottom-left corner is in [0,0]. The center of rotation now here at [0.3, 0.125, 0.2]
  # correct center of rotation:
  origin [ 0.22 0.0 0.0 0.0]
  gui_nose 1
  color "blue"

  # kinematics
  drive "car"
  wheelbase 0.5 # distance between both axles

  # [ xmin xmax ymin ymax zmin zmax amin amax ]
  velocity_bounds [-3 3 -1 1 -1 1 -90 90 ]
  acceleration_bounds [-1 1 -1 1 -1 1 -90 90]

  # spawn sensors
  laser(pose [ -0.1 0.0 -0.11 0.0 ])
)