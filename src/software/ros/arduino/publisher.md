# Publisher #

To use ROS in an Arduino script you first have to include some libraries :

```cpp
#include <ros.h>
#include <std_msgs/Type.h>
```

You have to replace «Type» with the type of the std message you are using. Here are the different types of std_msgs : <http://wiki.ros.org/std_msgs>.

You can also use different kind of messages. For instance the ultrasound sensors use :
```cpp
#include <sensor_msgs/Range.h>
```

Then you have to start a ROS node (takes care of serial port communications, allows to create publishers and subscribers) with this line :
```cpp
ros::NodeHandle nh;
```

You can now create a message object that you will later fill with data and this is the message you will finally publish :
```cpp
std_msgs::String msg;
```

where msg is the name of the object. And of course you can choose the type of the message depending on the type of the transferred message.


Then you have to choose if you want to create a publisher, a subscriber or even both.

To create a publisher :
```cpp
ros::Publisher chatter("chatter", &msg);
```

This tells that we are going to be publishing a message of the type of the msg variable (with std_msgs::String type) on the topic «chatter». This lets the master tell any nodes listening on «chatter» that we are going to publish data on that topic.

You can now add lines in the void setup() of the Arduino script. You first initialize the node :
```cpp
nh.initNode()
```
Next, the following call connects to the master to publicize that the node will be publishing messages on the given topic :
```cpp
nh.advertise(chatter);
```

So finally this is what the void setup() in your Arduino code should look like :
```cpp
void setup()
{
   nh.initNode();
   nh.advertise(chatter);
}
```
To fill the message object with the data you just have to do this :
```cpp
msg.data = hello;
```
where hello is the data you put in your message and hello must have the same type as the msg variable.

You can then publish you message on the topic. To do so you can either publish it once or continually in the void loop() of your Arduino code like this :
```cpp

void loop()
{
   chatter.publish( &str_msg );
}
```

Finally you have to add a last line of code in the loop that will call all the callbacks waiting to be called at that point in time :
```cpp
nh.spinOnce();
```

Now that you know how each part of the code works you can test a «Hello World» example available on the wiki.ros.org website <http://wiki.ros.org/rosserial_arduino/Tutorials/Hello%20World> :
```cpp
/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */
// Use the following line if you have a Leonardo or MKR1000 
//#define USE_USBCON 

#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";

void setup()
{
  nh.initNode();
  nh.advertise(chatter);
}

void loop()
{
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(1000);
}
```
You have to upload this code on the Arduino board before connecting it to the Raspberry and before starting the test on ROS.


# Test on a Raspberry #

To test it you first have to connect the Arduino board to the Raspberry. Then you launch the terminal and execute the following commands : 

on a first window you type : 
```
roscore
```

on another window : 
```
rosrun rosserial_python serial_node.py /dev/ttyUSB0 
```
/dev/ttyUSB0 is the usb port the Arduino is connected to so you’ll have to change the «USB0». You can find the name of the port tty* in the Arduino IDE or you can find it in the terminal by typing :
```
ls /dev/tty*
```
when you plug the Arduino in the Raspberry port, you can execute this previous command to see which port has been added and thus know which one is the Arduino board.

Finally you can see what the Arduino is publishing on a specific topic by typing on another terminal window :
```
rostopic echo /topic_name
```

where you have to replace topic_name by the topic your Arduino is publishing on. In the example above, the «hello world!» is published on the topic «chatter». So you can type :
```
rostopic echo /chatter
```
