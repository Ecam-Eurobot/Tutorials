# Folder structure

Here is the general structure of our project for Eurobot 2018.
```bash
Eurobot-2018
│   README.md
│   LICENSE    
│   .gitignore
└───arduino
│   
└───documentation
|
└───ros_packages
|   └───differential-drive
│   |
|   └───differential_driver
│   |
|   └───ecam_msg
│   |
|   └───mecanum
│   |
|   └───navigation
│   |
|   └───robot_2018
│   |
|   └───robot_description
│   |
|   └───sensors
│   |
|   └───strategy
```

The different folders:
* **arduino**: This folder contains all the arduino codes used within the 2 robots.
* **documentation**: The project documentation for the same year of Eurobot.
* **ros_packages**: Contains all our ros packages.
    * **differential-drive**: Provides some basic tools for interfacing a differential-drive robot with the ROS navigation stack. The intention is to make this independent of specific robot implementation.
    * **differential_driver**: package for the differential drive of Minus.
    * **ecam_msg**: contains custom messages for robots.
    * **mecanum**: package for the mechanic wheels of Cortex.
    * **robot_2018**: package that manages the launch of 2 robots.
    * **robot_description**: package that describes robots for simulation with rviz.
    * **sensors**: packages for ultrasound management.
    * **strategy**: package for the strategies chosen by the 2 robots.

> For more information about what is a [package](software/ros/basics/packages.html) in ROS.



