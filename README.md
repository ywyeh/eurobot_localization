# eurobot_localization

This repo provides a localization system based on EKF which fusion the data from Odom and 2D-LiDAR.

`ekf.cpp`: implement the ekf localization algorithm in my master thesis which modified from Probabilistic Robotics p. 217.

### /config

different map is corresponds to different beacon position (x, y coordinates) and yaml file.

`ekf_dit.yaml`: the playground in DIT Robotics (R314)

`ekf_stage.yaml`: the playground in stage simulator, weigth = 2 m and height = 3 m.

`ekf_stage_mapf.yaml`: the playground for the experiment Banny Wang needs.

`ekf_realworld.yaml`: the playground for the final edition experiment in my master thesis.
