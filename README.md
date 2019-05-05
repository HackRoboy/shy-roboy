# Shy Roboy
#### Always respect personal space!
HackRoboy Spring 2019 project. Roboy avoids contact with people and tells them to leave.
If they don't respect personal space, it even calls the police!

# ROS topics

topic | datatype | description
--- | :---: | ---
`shy_roboy/nearest_distance` | Float32 | Mean of the closest points to camera (depends on the threshold)
`shy_roboy/state` | Int8 | __IDLE:__ 0<br>__OCCURED:__ 1<br>__SHOUT:__ 2<br>__WATCH:__ 3

# Program States

<img src="images/roboy_state_machine.png">

# ROS Nodes

- Listening on the depth image and outputting average distance of the closest objects.
- Listening on the average distance and shouting when below some threshold.
- Listening shouting and showing lights on LED.
- Listening shouting and showing angry emotion.
- Listening on the average distance and moving head backwards proportional to some distance.

