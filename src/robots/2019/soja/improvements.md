# Things that weren't quite finished and other research directions for next year(s)

## move.py
As said earlier, the move package is something that has to be redone from scratch.

### use of feedback message
The first thing that should be done to have a basic, functioning robot is to try to edit the python file so that it only sends the next command when a message is received on the `cmd_feedback` topic.

### wait for start instruction
After that it could listen for a message published somewhere by the node managing the starting rope before sending the first command.

### ROS actions
Once all that is working and the robot can move around like it should, only then can you begin to research what ROS actions are and try to use that instead of topics to send commands to the motor-driving Arduino. One good thing with ros actions is that they provide feedback and are preemptible as where we had to implement those features in our own (objectively worse) way to achieve a similar functionality with topics.

The way it is implemented now required us to fake an ultrasound sensor to stop the robot after the match time was over. This is obviously a bodge and could be implemented in a much better way.
