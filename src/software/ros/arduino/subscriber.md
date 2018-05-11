# Subscriber #

To try this tutorial on you Arduino board you have to connect a Led. The pin used here is the 13 but of course you can change it in the code if you want to use another one.

As in the Arduino Publisher Tutorial you first have to include the libraries. One of them depends on the message you are transferring with ROS. In the following example we will choose an empty message. This means that the message doesn’t contain anything. So when the subscriber receives the message, it doesn’t react depending on its content but it only reacts because a message has been sent. As the type of the std_msg is «Empty», you have to include the following lines at the beginning of your Arduino code :
```cpp
#include <ros.h>
#include <std_msgs/Empty.h>
```
Here again, like the Arduino Publisher Tutorial code, you have to start a ROS node with this line :
```cpp
ros::NodeHandle nh;
```
Then you initiate the callback function where you have to specify the name of the callback (messageCb), the type of the message (std_msgs::Empty) and the name of the message (toggle_msg which contains the transferred message). The callback is the function called each time you receive a message. If toggle_msg had a content you could use it in the callback but it is not the case here as the message is of type «Empty». 
```cpp
void messageCb( const std_msgs::Empty& toggle_msg){
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
}
```

The following line of code is used to instantiate the subscriber. To do so you have to specify two arguments : the topic name (we chose in this example : «toggle_led») and the callback function we defined previously (messageCb).
```cpp
ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb );
```
As this is a blink led example, in the setup part of your Arduino code you first have to use the number 13 pin as an output.
```cpp
pinMode(13, OUTPUT);
```

Then for the ROS part of the setup you have to first initialize the node :
```cpp
nh.initNode();
```
And subscribe to the node you want to :
```cpp
nh.subscribe(sub);
```
So finally this is what the void setup() in your Arduino code should look like :
```cpp
void setup()
{
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}
```
Finally you just have to add in the void loop of your Arduino code the following lines where the spinOnce passes arguments to the callback :
```cpp
void loop()
{
  nh.spinOnce();
  delay(1);
}
```
Now that you know how each part of the code works you can test the blink led example available on the wiki.ros.org website <http://wiki.ros.org/rosserial_arduino/Tutorials/Blink> :
```cpp
/*
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */
#include <ros.h>
#include <std_msgs/Empty.h>

ros::NodeHandle nh;

void messageCb( const std_msgs::Empty& toggle_msg){
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
}

ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb );

void setup()
{
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();
  delay(1);
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
when you plug the Arduino in the Raspberry usb port, you can execute this previous command to see which port has been added and thus know which one is the Arduino board.

Finally, in order to send a single message to the Arduino, you can publish on a specific topic by typing on another terminal window :
```
rostopic pub toggle_led std_msgs/Empty --once
```

where toggle_led is the topic you are publishing on, std_msgs/Empty is the type of the message and we use «—once» to publish this message only once.
