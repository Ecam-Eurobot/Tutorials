This part contains the documentation on:

  - Total match time with ROS
  - The communication with the arduino and the raspberry pi
  - The launch files used

## Note
First of all you have to activate the *pigpiod* library, because it is disabled by default, by typing in the command line :

```sh
$ sudo systemctl enable pigpiod
```

Then, if we took the image of the site [ Ubiquity Robotics][ubiquity] as it was the case for 2019, we must type these two commands in CLI, the first one deactivates roscore, this one will be activated later with the launch files and the second one deactivates the ROS codes of the Ubiquity robots:

```sh
$ sudo systemctl disable roscore && sudo systemctl disable magni-base
```

To launch the launch files you must first install the ROS package below:

```sh
$ sudo apt-get install ros-$distro-robot-upstart
```

For each copy/paste of a code, it is imperative to not to forget to give the permissions on the Python files:

```sh
$ sudo chmod +x *.py
```

Then:

```sh
$ catkin_make
```

Finally, you must activate the launch file of startup_conf if you use it:

```sh
$ rosrun robot_upstart install start/launch/startup_conf.launch
$ sudo systemctl daemon-reload && sudo systemctl enable start
```

#### VNC
To activate VNC and connect to the raspberry pi UI you must activate the VNC server, then with VNC Viewer set: 0 after the IP address and vncserver as password.

```sh
$ sudo x11vnc -auth guess -forever -loop -noxdamage -repeat -rfbauth /home/ros/.vnc/passwd -rfbport 5900 -shared -display :0
```

#### Total match time with ROS

First we add these few lines to add the start switch interrupt, then the start function will be called.

```python
startInterrupt = pi.callback(pin_start, pigpio.EITHER_EDGE, start)
```

Once the start function is called, the game has started, this function calls up the ROS timer. This one takes as a parameter the time of the game, it can be retrieved via the *game.yaml* file. Once this timeout has passed, the function end_of_game_cb is called. It sends a'stop' message that simply stops the motors.

```python
def end_of_game_cb(event):
    # Stop the game
    rospy.loginfo("End of game")
    rospy.loginfo(event)

    pub_run.publish("stop")

# To start the timer once
publish = True

def start(gpio, level, tick):
    global publish
    rospy.sleep(0.2)
    if pi.read(pin_start) and publish:
        pi.write(pin_strategy_feedback_1, 1)
        pub_run.publish("start")
        rospy.Timer(rospy.Duration(90), end_of_game_cb, oneshot=True)
        publish = False
```

#### Communication with arduino and raspberry pi

For interruptions, the arduino sends all the data related to the sensors, the raspberry pi receives this data and if a value is less than 10 then we send a stop message to the arduino in which there is a subscriber to this topic who stops the motors once the message is received.

The arduino code subscriber that can be found in *motor.ino*

```cpp
ros::Subscriber<std_msgs::String> sub_stop("cmd_stop", &stop_cb );
```

Once the message is received, this function is called:
```cpp
void stop_cb(const std_msgs::String& msg) {
    if (strcmp(msg.data, "stop") == 0) {
        stop_condition = true;
        stopwheels();
    }
    else
        stop_condition = false;
}
```

#### The launch files used

We used launch files for the startup (for the tutorial related to launch files, we can refer to last year's tutorial).

If we have 3 arduinos, to launch all 3 at the same time, we can use this script.

```xml
<launch>
    <node name="Arduino0" pkg="rosserial_python" type="serial_node.py">
    	<param name="port" value="/dev/ttyACM0" />
    	<param name="baud" value="57600" />
    </node>
    <node name="Arduino1" pkg="rosserial_python" type="serial_node.py">
            <param name="port" value="/dev/ttyACM1" />
            <param name="baud" value="57600" />
    </node>
    <node name="Arduino2" pkg="rosserial_python" type="serial_node.py">
            <param name="port" value="/dev/ttyACM2" />
            <param name="baud" value="57600" />
    </node>
</launch>
```

[//]: # (These are reference links used in the body of this note and get stripped out when the markdown processor does its job. There is no need to format nicely because it shouldn't be seen. Thanks SO - http://stackoverflow.com/questions/4823468/store-comments-in-markdown-syntax)


   [ubiquity]: <https://downloads.ubiquityrobotics.com/pi.html>