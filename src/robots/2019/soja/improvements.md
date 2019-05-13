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

## Development process

### Git & github
Git and especially github are magnificent tools when used correctly. At the start of the project, it would be a good idea to make sure everyone knows how to clone, fork and contribute to a repository. Everyone involved in the project should know what a branch is and what can be done with it.

### Github projects
This year we used Trello and microsoft Teams to plan the whole project out and to share files. Github now proposes similar features directly integrated with the codebase. You can even assign certain cards to issues linked to specific files or commits. It could be worth the time to investigate if and how such a tool could be of any use to manage a project like this (spoiler: it probably is). In any case, using Teams to share code is certainly a very bad idea. That is the exact reason source control was invented in the first place. It could however still be useful to share very large binary files like VM images or to store backups of the raspberry pi's sd card.
