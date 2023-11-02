define kinectscan ranger
(
  sensor( 			
    range [ 0.0  10.0 ]
    fov 60.0
    samples 241
  )

  # generic model properties
  color "black"
  size [ 0.05 0.05 0.05 ]
)

define turtlebot position
(
  size [0.35 0.35 0.35]    # Actual size
  origin [0 0 0 0]    # The pioneer's center of rotation is offset from its center of area
  
  gui_nose 1    # Draw a nose on the robot so we can see which way it points
  obstacle_return 1    # Can hit things
  laser_return 1    # Can reflect beams
  
  drive "diff"    # Differential steering model
  
  # This block approximates a circular shape of a Robot 
  block( points 16
      point[0] [ 0.225 0.000 ]
      point[1] [ 0.208 0.086 ]
      point[2] [ 0.159 0.159 ]
      point[3] [ 0.086 0.208 ]
      point[4] [ 0.000 0.225 ]
      point[5] [ -0.086 0.208 ]
      point[6] [ -0.159 0.159 ]
      point[7] [ -0.208 0.086 ]
      point[8] [ -0.225 0.000 ]
      point[9] [ -0.208 -0.086 ]
      point[10] [ -0.159 -0.159 ]
      point[11] [ -0.086 -0.208 ]
      point[12] [ -0.000 -0.225 ]
      point[13] [ 0.086 -0.208 ]
      point[14] [ 0.159 -0.159 ]
      point[15] [ 0.208 -0.086 ]
      z [0 1]
  )
  
  kinectscan(pose [ 0.050 0.000 -0.1 0.000 ])  
)

define floorplan model
(
  # sombre, sensible, artistic
  color "gray30"

  # most maps will need a bounding box
  boundary 1

  gui_nose 0
  gui_grid 0

  gui_outline 0
  gripper_return 0
  fiducial_return 0
  laser_return 1
)
