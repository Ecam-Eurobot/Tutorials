# rpi/odom.cpp

This file is part of the odom package on the raspberry pi. Its purpose is to listen to messages published on the `encoder_ticks` topic and use that information to craft an `odom` message that we can use to localize our robot on a map using [RViz](software/ros/advanced/rviz.md).

The whole code is *heavily* inspired by [this question](https://answers.ros.org/question/11973/gathering-wheel-encoder-data-and-publishing-on-the-odom-topic/) on the ROS forum.
