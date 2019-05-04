# shy-roboy
HackRoboy Spring 2019 project. Roboy avoids contact with people and tells them to leave.

# ROS topic

topic | datatype | description
--- | --- | ---
`shy_roboy/nearest_distance` | Float32 | some mean of closest distance to camera
`shy_roboy/state` | Int8 | 0: IDLE, 1: OCCURED, 2: SHOUT, 3: WATCH


# ros nodes

- listening on the depth image and outputting one distance value
- listening on the depth and shouting when below some threshold
- listening on the depth and moving head backwards proportional to some distance 
- tbc
