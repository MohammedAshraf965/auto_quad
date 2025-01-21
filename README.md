# auto_quad
This repo will be used for auto_quad, an **autonomous quadcopter** that can be used for indoor and outdoor navigation and mapping

# How it works
The quadcopter will utilize computer vision and slam for navigation and mapping. Since the pi 4B would not be able to run cv algorithms in real-time, it will publish the image as messages that can be subscribed from by the local machine that is powerful enough to run cv. Also, the mapping will be computed on the local machine, meaning that the pi will publish the readings it gathers, which will then be computed in the local machine

Arudpilot will also be used, though the GPS would not work for indoor localization

# Components:
-- F450 quadcopter frame
-- Flight computer (RaspberryPi 4B 4Gb)
-- Flight computer (Speedybee F405 V3 stack)
-- Pi HQ camera (with 6mm cs mount)
-- Binocular camera module (for depth and as the front camera)
-- Slamtec RPLidar C1 (for mapping)
-- 4 BLDC (1000 kv)
-- 10x47 propellers
-- 3A power bank (powersupply for the flight computer)
-- 2200 mah 3S lipo battery (for the flight controller and the motors)
-- Mobile Wifi module

# To do list:
- [X] Test the cameras and the lidar module 
- [ ] Create/Import the components' urdf
- [ ] Visualize the model in gazebo
- [ ] Implement SLAM (using slam_toolbox)
- [ ] Test indoor mapping
- [ ] Create the control directory
- [ ] Test indoor navigation
- [ ] ROS2 with ardupilot
