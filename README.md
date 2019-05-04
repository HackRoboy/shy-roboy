# shy-roboy
HackRoboy Spring 2019 project. Roboy avoids contact with people and tells them to leave.

# ROS topic

topic | datatype | description
--- | --- | ---
`shy_roboy/nearest_distance` | Float32 | Mean of the closest points to camera (depends on the threshold)
`shy_roboy/state` | Int8 | 0: IDLE, 1: OCCURED, 2: SHOUT, 3: WATCH (a person was altready asked to leave, but didn't leave yet) 


# ros nodes

- listening on the depth image and outputting one distance value
- listening on the depth and shouting when below some threshold
- listening on the depth and moving head backwards proportional to some distance 
- tbc
