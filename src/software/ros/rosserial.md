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

**Code to upload on the Arduino before launching rosserial :**

```cpp
#include <ros.h>
#include <sensor_msgs/Range.h>
#include <NewPing.h>

#define TRIGGER_PINR  5   //back
#define ECHO_PINR    4   

#define TRIGGER_PINL  7   //front
#define ECHO_PINL     6  

#define TRIGGER_PINB  12   //left
#define ECHO_PINB     13 

#define TRIGGER_PINF  11   //right
#define ECHO_PINF     10 

#define MAX_DISTANCE 300 // Maximum distance we want to ping  

NewPing sonarL(TRIGGER_PINL, ECHO_PINL, MAX_DISTANCE); // back us 
NewPing sonarR(TRIGGER_PINR, ECHO_PINR, MAX_DISTANCE); // front us
NewPing sonarB(TRIGGER_PINB, ECHO_PINB, MAX_DISTANCE); // left us
NewPing sonarF(TRIGGER_PINF, ECHO_PINF, MAX_DISTANCE); // right us

ros::NodeHandle  nh;

sensor_msgs::Range range_msg_rear;
sensor_msgs::Range range_msg_front;
sensor_msgs::Range range_msg_left;
sensor_msgs::Range range_msg_right;
ros::Publisher pub_range1("ultrasound_rear", &range_msg_rear);
ros::Publisher pub_range2("ultrasound_front", &range_msg_front);
ros::Publisher pub_range3("ultrasound_left", &range_msg_left);
ros::Publisher pub_range4("ultrasound_right", &range_msg_right);
 

char frameid[] = "base_link";

long duration;
 float tmp;

void setup()
{
  nh.initNode();
  nh.advertise(pub_range1);
  nh.advertise(pub_range2);
  nh.advertise(pub_range3);
  nh.advertise(pub_range4);

  range_msg_rear.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg_rear.header.frame_id =  "ultrasound_rear";
  range_msg_rear.field_of_view = 0.3665;  // fake
  range_msg_rear.min_range = 0.0;
  range_msg_rear.max_range = MAX_DISTANCE;
  
  range_msg_front.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg_front.header.frame_id =  "ultrasound_front";
  range_msg_front.field_of_view = 0.3665;  // fake
  range_msg_front.min_range = 0.0;
  range_msg_front.max_range = MAX_DISTANCE; 
   
  range_msg_left.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg_left.header.frame_id =  "ultrasound_front";
  range_msg_left.field_of_view = 0.3665;  // fake
  range_msg_left.min_range = 0.0;
  range_msg_left.max_range = MAX_DISTANCE;  
  
  range_msg_right.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg_right.header.frame_id =  "ultrasound_right";
  range_msg_right.field_of_view = 0.3665;  // fake
  range_msg_right.min_range = 0.0;
  range_msg_right.max_range = MAX_DISTANCE;
}

long range_time;

void loop()
{
  //publish the adc value every 50 milliseconds
  //since it takes that long for the sensor to stabilize
  if ( millis() >= range_time ){
    tmp=sonarL.ping_cm();
    range_msg_rear.range = tmp/100;
    range_msg_rear.header.stamp = nh.now();
    pub_range1.publish(&range_msg_rear);

    tmp=sonarR.ping_cm();
    range_msg_front.range = tmp/100;
    range_msg_front.header.stamp = nh.now();
    pub_range2.publish(&range_msg_front);
    
     tmp=sonarB.ping_cm();
    range_msg_left.range = tmp/100;
    range_msg_left.header.stamp = nh.now();
    pub_range2.publish(&range_msg_left);

    tmp=sonarF.ping_cm();
    range_msg_right.range = tmp/100;
    range_msg_right.header.stamp = nh.now();
    pub_range4.publish(&range_msg_right);

    range_time =  millis() + 50;
  }
  nh.spinOnce();
}


```
