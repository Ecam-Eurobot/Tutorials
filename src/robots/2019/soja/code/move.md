# Move package

The move package as it is in the github repository is not quite functional yet. Its purpose was to have a sequence of actions to execute in order ie. dirve forward for 50cm, turn 90° counterclockwise, drive 20 cm forward and activate the arm.

The first thing that we wanted to do is to determine a set of absolute positions to drive to instead of having to tell each action. After losing way too much time trying to understand ROS actions and how to convert a current and target position in the form of an `odom` message to a set of instructions to be executed by the robot, we came to realise it would be much, *much* easier to directly have a list of actions to execute. This is exactly what we ended implementing. Unfortunately for us the tournament was only a few days away and we still had many other problems to solve. All this led to some really rushed and unstable code that ended up saving our ass and managed to score just a few points (and even win one match!).

That code has long been burned to the ground and will never be spoken of again. So after that, we were so embarrassed that we tried to rewrite the code a bit better. That code never got quite finished and it is the one that you can find [here](https://github.com/Ecam-Eurobot/Eurobot-2019/blob/grand_robot/ros_packages/move/src/move.py).

The general idea is that we should listen on some topic for a start message, after wich we send the first command to execute. We then wait for a feedback from the arduino telling us when that command has been successfully executed. Once we receive that signal, we can safely send the next instruction.
