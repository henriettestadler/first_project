## Overview

This ROS project was created by Mathias Askeland, Felipe Naranjo Amórtegui and Henriette Stadler for the Robotics class at Polimi.
This project includes three ROS nodes that process GPS, steering, and speed data to generate odometry information and calculate sector times. In the following the three ROS nodes (odometry, gps_odometry, sector_times) are explained. 
The project can be found online in github: https://github.com/henriettestadler/first_project/tree/main


## Odometry
- **Input:**  
  `/speedsteer` (`geometry_msgs/PointStamped`)
    - `x`: Steering angle at the steering wheel (degrees)  
    - `y`: Speed (km/h)

- **Output:**  
  `/odom` (`nav_msgs/Odometry`)  
  `tf` transform from `odom` to `vehicle`

- **Function:**  
  Uses a simple kinematic model to estimate vehicle position and orientation based on speed and steering inputs. The coordinate system used is x=forward, y=left and z=up with respect to the driver.



## GPS_Odometry
- **Input:**  
  `/swiftnav/front/gps_pose` (`sensor_msgs/NavSatFix`)
  
- **Output:**  
  `/gps_odom` (`nav_msgs/Odometry`)  
  `tf` transform from `odom` to `gps`

- **Function:**  
  Converts raw GPS coordinates (latitude, longitude, altitude) into odometry data in the ENU coordinate frame and broadcasts the transform.



## Sector_Times
- **Input:**  
  `/swiftnav/front/gps_pose` (`sensor_msgs/NavSatFix`)  
  `/speedsteer` (`geometry_msgs/PointStamped`)

- **Output:**  
  `/sector_times` (`first_project/sector_times`) — custom message

- **Function:**  
  Monitors vehicle's position and timing to determine how long it spends in predefined GPS sectors. The GPS sectors were defined approximately by eye. The image of the track with the intended sectors given in the challenge.pdf was used as a reference and GPS points were manually selected near the desired boundaries in RViz to define the sector edges. A radius of 5m was defined as "sector boundary".

## Launch.launch
Lastly, to start all three nodes, a launch file was created. Besides the three nodes, rviz is also opened. Since for this project a docker container is used, some of us needed to use noVNC. If this is the case, line 4 of the file has to be uncommented.
