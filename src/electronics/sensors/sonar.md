# Sonar #

A sonar is a sensor allowing you to measure the distance to an obstacle thanks to high frequency waves. To keep correct measured values it is better to use the sensor we chose between 2cm and 400cm. The model of our sonar is HC-SR04. There are 4 pins on this sensor : TRIG, ECHO, VCC and GND (as you can see on the picture below).

![img](img/electronics/sonar/HCSR04.png)

It contains a transmitter and a receiver.
The distance measure is pretty simple. Firstly, you have to send a signal on the TRIG pin so the sonar emits a wave of 8 pulses at 40kHz. Then the ECHO pin is set to the «HIGH» level until the emitted signal goes to the obstacle and comes back to the sensor (as you can see on the following picture).

![img](img/electronics/sonar/ultrasonic_waves.png)

Of course, you have to connect the sensor to a microcontroller. You can connect the VCC to the 5V pin of your board, the GND to the ground pin and the TRIG and ECHO to digital pins of your controller as illustrated on the picture below.

![img](img/electronics/sonar/sonar_pins.png)

Finally it is also important to know that the measuring angle with the HC-SR04 sonar is 30 degrees.

All these information are also explained in the following websites :

<https://www.gotronic.fr/pj2-hc-sr04-utilisation-avec-picaxe-1343.pdf>

<https://wiki.mchobby.be/index.php?title=HC-SR04>

<http://web.eece.maine.edu/~zhu/book/lab/HC-SR04%20User%20Manual.pdf>

Images references :

<http://geii.iut-troyes.univ-reims.fr/wikigeii/index.php?title=Fichier:Working-of-HC-SR04-Ultrasonic-Sensor.jpg>

<http://www.instructables.com/id/HC-SR04-Ultrasonic-Sensor-With-Raspberry-Pi-2/>

<https://randomnerdtutorials.com/complete-guide-for-ultrasonic-sensor-hc-sr04/>

# Tutorial #

This tutorial explains the code to use and transfer sonar sensors information with ROS.

For the «Cortex» robot we used four sensors, one on each side so that’s what the final code presented at the end of this tutorial is based on.

Firstly, in this code we are going to include the same libraries as shown in the Arduino Publisher Tutorial so you can refer to that part for the explanations.
An additional useful library used for the sensors is the NewPing.h. So here are all the necessary libraries :
```cpp
#include <ros.h>
#include <sensor_msgs/Range.h>
#include <NewPing.h>
```
Then you have to define the pins for each sensor. In this case, the ultrasound sensor has two pins (Trigger ans Echo). Here is an exemple with the sensor located on the right of the robot:
```cpp
#define TRIGGER_PINR  5
#define ECHO_PINR    4   
```
You also have to specify the maximum distance at which you want the sensor to still be able to detect :
```cpp
#define MAX_DISTANCE 300
```
Now as we are using the NewPing library you can simply create an ultrasound sensor object by doing this :
```cpp
NewPing sonarR(TRIGGER_PINR, ECHO_PINR, MAX_DISTANCE);
```
where you specify the pins and the maximum distance defined previously.

Moreover as mentioned in the Arduino Publisher Tutorial you specify the type of the ultrasound message and the name you want to assign to it :
```cpp
sensor_msgs::Range range_msg_right;
```
You also have to add the following line which has also been explained in the Arduino Publisher Tutorial :
```cpp
ros::Publisher pub_range4("ultrasound_right", &range_msg_right);
```
We then have to fill each sonar object with the initialisation information associated :
```cpp
range_msg_right.radiation_type = sensor_msgs::Range::ULTRASOUND;
range_msg_right.header.frame_id =  "ultrasound_right";
range_msg_right.field_of_view = 0.3665;  // fake
range_msg_right.min_range = 0.0;
range_msg_right.max_range = MAX_DISTANCE;
```
Now we can add parts to the void loop of the Arduino code. The most important and useful one specifies the distance to an obstacle :
```cpp
range_msg_right.range = tmp/100;
```
We can then publish that information about the distance (you can again refer to the Arduino Publisher Tutorial for the publishing part) :
```cpp
pub_range4.publish(&range_msg_right);
```
Here is the whole code for four ultrasound sensors based on the one presented in this link :
<https://www.youtube.com/watch?v=gm3e-51ohgQ>  
Moreover the code below is available in the Github repo of Ecam Eurobot : <https://github.com/Ecam-Eurobot/Eurobot-2018/blob/ultrasound/arduino/sonar.ino>  

You can find the tutorial in the Software chapter, ROS and Arduino part to test if you succesfully receive information from the sonar with rosserial.

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
