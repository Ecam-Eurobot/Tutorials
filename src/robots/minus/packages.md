# Packages

## Differential-drive

Having no time to implement ourselves, a library to manage the differential navigation, we look for an existing library compatible with ROS.

This package provides some basic tools for interfacing a differential-drive robot with the ROS navigation stack.

![Schema](http://wiki.ros.org/differential_drive?action=AttachFile&do=get&target=differential_drive_overview.png)

The library therefore needed only 2 input information, the return of the encoders of the left wheel (`lwheel`) and right (`rwheel`).
At the exit, we received the speeds of the 2 wheels, `lmotor` and `rmotor`.

#### Setting up the differential-drive package PID controller

The configuration of the package is done via a launchfile containing all the necessary parameters such as `Kp`, `Ki`, `Kd`, etc.

This configuration is in the file
`ros_packages/differential_driver/launch/minus.launch`

```xml
<launch>
  <rosparam param="ticks_meter">16400</rosparam>	
  <node pkg="differential_drive" type="diff_tf.py" name="diff_tf">
      <rosparam param="base_width">0.353</rosparam>
      <rosparam param="encoder_min">-1000000</rosparam>
      <rosparam param="encoder_max">1000000</rosparam>
  </node>
  <node pkg="differential_drive" type="pid_velocity" name="lpid_velocity">
      <remap from="wheel" to="lwheel"/>
      <remap from="motor_cmd" to="lmotor_cmd"/>
      <remap from="wheel_vtarget" to="lwheel_vtarget"/>
      <remap from="wheel_vel" to="lwheel_vel"/>
      <rosparam param="Kp">25</rosparam>
      <rosparam param="Ki">10</rosparam>
      <rosparam param="Kd">0</rosparam>
      <rosparam param="out_min">-254</rosparam>
      <rosparam param="out_max">254</rosparam>
      <rosparam param="rate">20</rosparam>
      <rosparam param="timeout_ticks">10</rosparam>
      <rosparam param="rolling_pts">5</rosparam>
      <rosparam param="encoder_min">-1000000</rosparam>
      <rosparam param="encoder_max">1000000</rosparam>
  </node>
  <node pkg="differential_drive" type="pid_velocity" name="rpid_velocity">
      <remap from="wheel" to="rwheel"/>
      <remap from="motor_cmd" to="rmotor_cmd"/>
      <remap from="wheel_vtarget" to="rwheel_vtarget"/>
      <remap from="wheel_vel" to="rwheel_vel"/>
      <rosparam param="Kp">25</rosparam>
      <rosparam param="Ki">10</rosparam>
      <rosparam param="Kd">0</rosparam>
      <rosparam param="out_min">-254</rosparam>
      <rosparam param="out_max">254</rosparam>
      <rosparam param="rate">20</rosparam>
      <rosparam param="timeout_ticks">10</rosparam>
      <rosparam param="rolling_pts">5</rosparam>
      <rosparam param="encoder_min">-1000000</rosparam>
      <rosparam param="encoder_max">1000000</rosparam>
  </node>

  <node pkg="differential_drive" type="twist_to_motors.py" name="twist_to_motors" output="screen">
    <remap from="twist" to="cmd_vel"/>
    <rosparam param="base_width">0.353</rosparam>
    <rosparam param="rate">20</rosparam>
  </node>
</launch>
```

This launch file assumes your robot:

* Takes commands for the wheel power on `/lmotor_cmd` and `/rmotor_cmd`

* Publishes the wheel encoder on `/lwheel` and `/rwheel`

* The base_width parameter should be set to the wheel spacing of your robot.

If this is not the case, you can change the mapping in the launch file.

As long as you meet these conditions, and the PID values are sane for your robot, you should be able to drive the robot.

> For more information about what is a [launchfile](software/ros/basics/launch.html).
A method for optimizing the PID parameters and calibrating the tick per meter is avalaible [there](http://wiki.ros.org/differential_drive/tutorials/setup).

## Sensors

This package allows us to manage the obstacle detection thanks to the data sent by the ultrasonic sensors connected to the arduino and the action to be performed.

The arduino publishes on various topics (`ultrasound_x`), the data it receives from the sensors.

The package reads the values and depending on the position of the robot (`move_base / feedback`) and the values sends a stop message on a topic (` obstacle / stop`).

 > For more information about what is a [publisher](software/ros/basics/pub.html), a [subscriber](software/ros/basics/sub.html) and [arduino with ros](software/ros/arduino/publisher.html).

 ## Strategy

In this package, we declare the strategy used by each robot in parameter files.

* **initial.yaml**: contains the position of the robots at the beginning of the game based on whether we are the red or green team.
* **game.yaml**: Contains game-specific information such as the duration of a game.
* **minus/cortex_actions.yaml**: contains the different actions that the robot must perform.

> For more information about what is a [parameter file](software/ros/basics/params.html).

