# ROS and Arduino

## What is rosserial ?

« Rosserial is a protocol for wrapping standard ROS serialized messages and multiplexing multiple topics and services over a character device such as a serial port or network socket.  
Rosserial provides a ROS communication protocol that works over your Arduino's UART. It allows your Arduino to be a full fledged ROS node which can directly publish and subscribe to ROS messages, publish TF transforms, and get the ROS system time.  
The rosserial protocol is aimed at point-to-point ROS communications over a serial transmission line. We use the same serialization/de-serialization as standard ROS messages, simply adding a packet header and tail which allows multiple topics to share a common serial link. »

These explanations come from the following websites :  
http://wiki.ros.org/rosserial  
http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup

Now you know what rosserial is used for, but before testing you have to install some packages.

## Rosserial installation

To install and use rosserial you have to run the terminal, first update apt-get which allows you to install packages :

```
sudo apt-get update
```
You can choose to install the packages one by one :

First type :
```
sudo apt-get install arduino
```
Then after the first installation :
```
sudo apt-get install ros-kinetic-rosserial
```
And then :
```
sudo apt-get install ros-kinetic-rosserial-arduino
```

Or you can directly install all the packages with one command in the terminal :
```
sudo apt-get install -y \
                arduino \
                ros-kinetic-rosserial-arduino \
                ros-kinetic-rosserial
```
For more information you can follow [installation of ros on arduino](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup)

## Arduino&Sensor ROS code

This tutorial explains the code to use and transfer sonar sensors information with ROS.

For the «Cortex» robot we used four sensors, one on each side so that’s what the final code presented at the end of this tutorial is based on.

Firstly, in this code we are going to include the same libraries as shown in the Arduino Publisher Tutorial so you can refer to that part for the explanations.
An additional useful library used for the sensors is the NewPing.h. So here are all the necessary libraries :
```cpp
#include <ros.h>
#include <sensor_msgs/Range.h>
#include <NewPing.h>
```
NewPing library isn't included in the Ros installation so you have to install it separatly [Installation of NewpPing librairy](https://playground.arduino.cc/Code/NewPing/#Download)

Then you have to define the pins for each sensor. In this case, the ultrasound sensor has two pins (Trigger ans Echo). Here is an exemple with the sensor located on the right of the robot:
```cpp
#define TRIGGER_PIN1  5
#define ECHO_PIN1    4   
```
You also have to specify the maximum distance at which you want the sensor to still be able to detect :
```cpp
#define MAX_DISTANCE 300
```
Now as we are using the NewPing library you can simply create an ultrasound sensor object by doing this :
```cpp
NewPing sonar1(TRIGGER_PINR, ECHO_PINR, MAX_DISTANCE);
```
where you specify the pins and the maximum distance defined previously.

Moreover as mentioned in the Arduino Publisher Tutorial you specify the type of the ultrasound message and the name you want to assign to it :
```cpp
sensor_msgs::Range range_msg;
```
You also have to add the following line which has also been explained in the Arduino Publisher Tutorial :
```cpp
ros::Publisher pub_range1("ultrasound_1", &range_msg);
```
We then have to fill each sonar object with the initialisation information associated :
```cpp
range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
range_msg.header.frame_id =  "ultrasound_right";
range_msg.field_of_view = 0.3665;  // fake
range_msg.min_range = 0.0;
range_msg.max_range = MAX_DISTANCE;
```
Now we can add parts to the void loop of the Arduino code. The most important and useful one specifies the distance to an obstacle :
```cpp
range_msg.range = tmp/100;
```
We can then publish that information about the distance (you can again refer to the Arduino Publisher Tutorial for the publishing part) :
```cpp
pub_range1.publish(&range_msg);
```
Here is the whole code for four ultrasound sensors based on the one presented in this link :
<https://www.youtube.com/watch?v=gm3e-51ohgQ>  
Moreover the code below is available in the Github repo of Ecam Eurobot : <https://github.com/Ecam-Eurobot/Eurobot-2018/blob/ultrasound/arduino/sonar.ino>  

**Code to upload on the Arduino before launching rosserial :**

```cpp
#include <ros.h>
#include <sensor_msgs/Range.h>
#include <NewPing.h>

#define TRIGGER_PIN1  5   //sensor 1
#define ECHO_PIN1    4   


#define MAX_DISTANCE 300 // Maximum distance we want to ping  

NewPing sonar1(TRIGGER_PIN1, ECHO_PIN1, MAX_DISTANCE); // back us

ros::NodeHandle  nh;

sensor_msgs::Range range_msg;

ros::Publisher pub_range1("ultrasound_1", &range_msg);

char frameid[] = "base_link";

long duration;
 float tmp;

void setup()
{
  nh.initNode();
  nh.advertise(pub_range1);

  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.header.frame_id =  "ultrasound_1";
  range_msg.field_of_view = 0.3665;  // fake
  range_msg.min_range = 0.0;
  range_msg.max_range = MAX_DISTANCE;

}

long range_time;

void loop()
{
  //publish the adc value every 50 milliseconds
  //since it takes that long for the sensor to stabilize
  if ( millis() >= range_time ){
    tmp=sonar1.ping_cm();
    range_msg.range = tmp/100;
    range_msg.header.stamp = nh.now();
    pub_range1.publish(&range_msg_rear);
    range_time =  millis() + 50;
  }
  nh.spinOnce();
}

```

## Rosserial sonar simple test

To test if you receive the messages published by the Arduino on the Rasperry Pi you have to do the following :  
To test it you first have to upload the code you can find at the end of this tutorial (which you can also find on the Github repo of the Ecam Eurobot : <https://github.com/Ecam-Eurobot/Eurobot-2018/blob/ultrasound/arduino/sonar.ino>). If you need further information and explanations for this code, you can refer to the Sonar Sensor Tutorial in the Electronics part. You can also use that tutorial for the wiring of the sonar on your Arduino. Of course you have to choose the pin numbers so they correspond to the ones declared in the code previously mentionned or you can directly change in the code the pin numbers yourself.  
Then you have to connect the Arduino board to the Raspberry. To do so you can simply connect them with the serial Arduino cable to the Raspberry USB port as shown on the figure below.  

![img](img/software/ros/arduino/rasp_arduino_connection.png)

Image reference :  
<http://www.instructables.com/id/Raspberry-Pi-Arduino-Serial-Communication/>

After that, you have to launch the terminal and execute the following commands :

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

Finally you can see what the Arduino is publishing on the topic of one of the ultrasound sensors, for example we will try here with the right ultrasound. You can see it by typing on another terminal window :

```
rostopic echo /ultrasound_right
```
