# Description

Dynamixels are servomotors with a working angle of 300° dispatched on values 0 to 1023.

![alt text](electronics/actuators/Dynamixels_SRC/AngleFonctionnement.png )

They also have the particularity of beeing able to be used as DC motor. They have a very good motor torque what makes them efficient for a big number of applications. In our robots, we used them for the tasks the robots had to realise during the competition as : 

*	Sort balls of different colors
*	Deploy an arm to push an object on wheels
*	Deploy a platform on multiple floors to liberate cubic blocks
*	Maintaining the blocks on every floor or not
*	Actionate a gear to move blocks

The cabling is made with JST 3 pin connectors from a servomotor to another.

Here is a link on amazon to order some : https://www.amazon.fr/ensembles-Micro-connecteur-Fiche-150mm/dp/B01DU9OY40/ref=sr_1_2?ie=UTF8&qid=1525997056&sr=8-2&keywords=jst+connecteur+3+pin&dpID=51oVa4jux4L&preST=_SY300_QL70_&dpSrc=srch

 ![alt text](electronics/actuators/Dynamixels_SRC/PinsConnections.png )

The Dynamixels have 2 locations for these connectors because it is possible to connect several servomotors in series and to control them with an addressing. These adresses are represented by unique ID for every dynamixel and it is possible to check with the software Dynamixel Wizard of Roboplus and a USB2Dynamixel.

![alt text](electronics/actuators/Dynamixels_SRC/USB2Dynamixel.png )

Thanks to this software, it is also possible to configurate the dynamixels (registers) like, among others, the baudrate of the servomotor and also to access a serie of informations in real time like the speed, the position, etc.

# Specifications

*	Weight : 53.5g (AX-12/AX-12+), 54.6g (AX-12A)
*	Dimension : 32mm * 50mm * 40mm
*	Resolution : 0.29°
*	Gear Reduction Ratio :  254 : 1
*	Stall Torque : 1.5N.m (at 12.0V, 1.5A)
*	No load speed : 59rpm (at 12V)
*	Running Degree :  0° ~ 300° or Endless Turn
*	Running Temperature : -5℃ ~ +70℃
*	Voltage : 9  ~ 12V (Recommended Voltage 11.1V)
*	Command Signal : Digital Packet
*	Protocol Type : Half duplex Asynchronous Serial Communication (8bit,1stop,No Parity)
* Link (Physical) : TTL Level Multi Drop (daisy chain type Connector)
*	ID : 254 ID (0~253)
*	Communication Speed : 7343bps ~ 1 Mbps
*	Feedback : Position, Temperature, Load, Input Voltage, etc.
*	Material : Engineering Plastic


# Dynamixel Wizard

*	Download and install the software "Roboplus" on the website www.robotis.com when going in Support>Download>Software>Roboplus
 
 ![alt text](electronics/actuators/Dynamixels_SRC/DW_Roboplus.png )
 
*	Launch the software and clic on Dynamixel Wizard 

 ![alt text](electronics/actuators/Dynamixels_SRC/DW_DynamixelWizard.png )

*	Insert the USB2Dynamixel in the USB port of the computer 
*	Connect the 3 pins JST connector in the USB2Dynamixel (TTL side) ansd the dynamixel to verifie or configure
*	Select « TTL » with the switch on the USB2Dynamixel
*	Supply the dynamixel separately in its working voltage (9-12V, recommanded 11,1V) with the second port of the dynamixel

 ![alt text](electronics/actuators/Dynamixels_SRC/DW_USB2Dynamixel.png )

*	Select the port of your computer where the USB2Dynamixel is connected

 ![alt text](electronics/actuators/Dynamixels_SRC/DW_Port.png )

*	Clic on "Open Port"

![alt text](electronics/actuators/Dynamixels_SRC/DW_OuvrirLePort.png ) 

* Make a basic research of the dynamixel to find its ID


*	It is also possible to make advanced research on other baudrates in case of the basic research is not working

*	Once the dynamixel found, selct it on the left side of the window 

*	The details of the informations of the dynamixel appears

*	in these details, it is possible to modify the configuration of the dynamixel like its ID, its communication speed (baudrate), its working speed, etc, or also to control it in real time.


# Examples of use

To use the dynamixels, it is necessary to download first the library "AX_12A_servo_library" that you can find on the github of eurobot here : https://github.com/Ecam-Eurobot/Tutorials/tree/master/src/electronics/actuators/Dynamixels_SRC

You'll have to extract it and place it in Documents/Arduino/libraries.

To configure the dynamixels, we'll use the Arduino software app.

This library possess 4 exemples of use of the dynamixel : 
-	Blink : example of control of the red led you can find in the back of every dynamixel
-	Move : example of control of the dynamixel in servomotor mode by giving it a position
-	EndlessTurn : example of use in DC motor mode (or wheel mode)
-	ReadRegister : usefull to dispaly the contents of the registers of the dynamixel

These examples are located in Fichier>Exemples>AX-12A on the Arduino app on your computer.

After selecting one of the examples, you have to include the library AX-12A without declaring a specific path because the library is located in the fold "libraries" of the fold "Arduino".

 ![alt text](electronics/actuators/Dynamixels_SRC/Exemples_Include.png )

After that you have to configure 3 lines in the code that respond to the needs of the configuration of the connected dynamixel you want to configure :

```
#define DirectionPin  (10u)
#define BaudRate      (1000000ul)
#define ID            (1u)
```

"DirectionPin" is used to indicate the communication direction of the dynamixel. 10u is for writing in the registers so you don't have to change the value.

"BaudRate" defines the communication speed used ( and configurate before with the Dynamixel Wizard)

"ID" represents the ID of the dynamixel that you checked or configured before for the addressing.


# Application with ROSserial

As explained in the chapter "The Cortex Robot", one dynamixel was used to control the separation of coloured balls (one colour representing 
dirty water and the other clean water). This dynamixel has three main positions:

* Initial_pos_purifier (blocked or closed) position
* Clean ball position (opens trap for green balls as in our earlier example)
* Dirty ball position (Open trap for orange balls as in our earluer example)

Another task that Cortex robot had was to push a bee for the foraging task. This meant we needed a mechanical hand that extended from the robot
in order to push the bee. We used a dynamixel for this action as well. This dynamixel has two main positions:

* initial_pos_bee (embedded inside robot)
* push_pos (extended from robot)

We calibrated these positions using the angle diagrams supplied in the data sheet and the good old "trial and error method".

## Schematic block Diagram

![alt text](mechanical/2018/BallGun_SRC/schemaBlock.png )

The communication between the ROS board (Rasberry pi with ROS installed) and the Arduino Uno board that controls the tasks of the Cortex robot takes place over
a USB serial communication better known as ROSserial. The concepts behind ROSserial concept are explored in detail in the section Software of this document. I
personally recommend reading that section to better understand the code presented here.

The three main actions of the Arduino board are:

* Separate dirty balls from clean balls (without colour detection and pusher in qualifying stages)  
* Push bee 
* Drive DC motor to shoot clean balls into home tank.

These actions are activated and controlled fully by the ROS board. That is, if the ROS board wants to push the bee, the Arduino board must be ready to receive
the message, activate the right actuators/actions in order to push the bee.

! Attention: It is imperative to program the dynamixels to the right baudrate first! This can be done using the USB2Dynamixel and Dynamixel Wizard software as
explained earlier. We had problems controlling the dynamixels via the ROS board at the default baudrate of 1000000 BPS. The communication speed that worked best
was the default baudrate used by ROS on the Raspberry pi, 57600 BPS. After setting the dynamixel in the wizard, all you have to do is change the baudrate 
in the code from 1000000ul to 57600ul. (Note: 1000000ul works fine when controlling the dynamixels with only the Arduino board, hmm). 

! Attention: These dynamixels use UART to communicate, if you use the Arduino Uno which only has one on board UART module then you will nt be able to use 
the serial monitor at the same time! DO NOT PUT SERIAL.BEGIN IN SETUP when you want to work with the dynamixels on an Arduino Uno. The best case would be to use a board
with more than one UART module, allowing you to control the dynamixels independent of the serial monitor.

## Code

```cpp
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <AX12A.h>

//Ax-12A IDs and baudrate
#define DirectionPin  (10u)
#define BaudRate     (57600ul)
#define ID1          (7u)  //water purification
#define ID2          (1u)  //bee

//Gun pin definitions
#define i1            5
#define i2            4
#define ena_pwm       9     //PWM pin at 490Hz 

//Water purification definitions
#define clean_ball_position 90
#define dirty_ball_position 916
#define initial_pos_purifier 512
//int initial_pos_purifier = 512;


//water purification declarations
int servo_speed = 500; //speed for ax-12a movement

//Dynamixel AX-12A definitions and global variables
//for control of valve used to separate/purify balls/water.
//@param valve_pos is controlled by ROS board.
int initial_pos_bee = 552;
int valve_pos;

//Start ROS handle.
ros::NodeHandle nh;

void moveBee( const std_msgs::Bool & position_msg){
  
  // Bool variable decides whether to push bee
  // or return to initial position. False = initial position
  // True = push bee position 
  bool pushbee = false;
  int pushbee_pos = 30;   // initial/push position for bee still needs to be calibrated
  
  pushbee = position_msg.data;
  if(pushbee){
    //ax12Move(ID2, pushbee_pos, servo_speed);
    ax12a.moveSpeed(ID2, pushbee_pos, servo_speed);
    }
  else {
    //ax12Move(ID2, initialpos, servo_speed);
    ax12a.moveSpeed(ID2, initial_pos_bee, servo_speed);
    }
}

void moveValve(const std_msgs::Int16 & pos_msg){
  
  valve_pos = pos_msg.data;
  if(valve_pos == 1){
    ax12a.moveSpeed(ID1, clean_ball_position, servo_speed);
    }
  else if (valve_pos == 2){
    ax12a.moveSpeed(ID1, dirty_ball_position, servo_speed);
    }
  else if(valve_pos == 3){
    ax12a.moveSpeed(ID1, initial_pos_purifier, servo_speed);
    }
}

void shootGun( const std_msgs::Int16 & dutycycle_msg){
   //Gun declarations. @param dutycycle (0-255) is controlled
  //by ROS board.
  int dutycycle = 0;
  dutycycle = dutycycle_msg.data;

  //direction
  digitalWrite(i1, LOW);
  digitalWrite(i2, HIGH); 

  //Drive
  analogWrite(ena_pwm, dutycycle);
}

//Move purifier to and fro in order to make
//sure all balls enter compartment
void shake(void);

ros::Subscriber<std_msgs::Bool> bee_ctrl("bee_control", &moveBee);
ros::Subscriber<std_msgs::Int16> ballseparator_ctrl("water_purification", &moveValve);
ros::Subscriber<std_msgs::Int16> gun_ctrl("gun_control", &shootGun);


void setup() {
    
  pinMode(ena_pwm , OUTPUT);
  pinMode(i1, OUTPUT);
  pinMode(i2, OUTPUT);
  
  //Start-up
  ax12a.begin(BaudRate, DirectionPin, &Serial);
  //Remove endless rotation
  ax12a.setEndless(ID1, OFF);
  ax12a.setEndless(ID2, OFF);
  //move into initial position
  ax12a.moveSpeed(ID1, initial_pos_purifier, servo_speed);
  ax12a.moveSpeed(ID2, initial_pos_bee, servo_speed);
  
  nh.initNode();
  
  nh.subscribe(bee_ctrl);
  nh.subscribe(ballseparator_ctrl);
  nh.subscribe(gun_ctrl);
}

//the loop contains is empty.
void loop() {
  nh.spinOnce();
  //delay(50);
  //shake();    
}


void shake(void)
{
    int dir = 0;
    int current_pos = ax12a.readPosition(ID1);
    
    if (valve_pos == 1){
    if (dir == 0){
      if (current_pos != clean_ball_position + 200){
        ax12a.moveSpeed(ID1, clean_ball_position + 200, servo_speed);
      }
      else {
        dir = 1;
      }
      
    }
    else if (dir == 1){
      if (current_pos != clean_ball_position){
        ax12a.moveSpeed(ID1, clean_ball_position - 200, servo_speed);
      }
      else {
        dir = 0;
      }
    }

  }
  if (valve_pos == 2){
    if (dir == 0){
      if (ax12a.readPosition(ID1) != dirty_ball_position + 200){
        ax12a.moveSpeed(ID1, dirty_ball_position - 200, servo_speed);
      }
      else {
        dir = 1;
      }
      
    }
    else if (dir == 1){
      if (ax12a.readPosition(ID1) != dirty_ball_position){
        ax12a.moveSpeed(ID1, dirty_ball_position + 200, servo_speed);
      }
      else {
        dir = 0;
      }
    }

  }
}
```

Note: ROS library packages are bulky and can get very big, adding more #includes or libraries to your code will add mean more global variables to your overall 
project. Make sure that your board, Uno, Nano etc. can support has enough memory space) for the large amount of global variables. I invite you to compile this
code and load it on an Arduino board. Check the memory details after loading, you will see that we're already almost at the limit ! 
Think about this for the future.

## How it works

### Libraries

Include all libraries that are necessary for the application but be sure to include ros.h first! Here we will need the AX12A library to control the dynamixel servo motor. We also need certain
source code from the ROSserial std_msgs (allows for communication over ROSserial) package. These source code files that we include will determine the type (int, foat64, String etc..)
of the messages that will be exchanged between the Arduino board and the ROS board. 

To control the position of the "bee dynamixel" we only need a true or false signal from ROS but for the "water purifier" we need three positions so we'll rather go for the type int here.
The choice is yours. 

Note: We had difficulties working with the std_msgs: Int8 and String when programming with the dynamixels so we settled with Int16 and Bool.
```
#include <ros.h>              //ROS packages
#include <std_msgs/Bool.h>     //ROS std messages needed for com
#include <std_msgs/Int16.h>
#include <AX12A.h>            //Dynamixel control
```

### Declarations and pin definitions
Declare as many variables as possible through the #define method in order to save global variable space on the Arduino Uno for example: 
```
//Gun pin definitions
#define i1            5
#define i2            4
#define ena_pwm       9     //PWM pin at 490Hz 

//water purification declarations
int servo_speed = 500;      //speed for ax-12a movement
```

Don't forget to create the ROS node!
```
//Start ROS handle.
ros::NodeHandle nh;    //Create a ROS node
```

### Create callback function

The Arduino board is seen as a node by the ROS board. In order to receive commands from it we need to subscribe to a topic to see what the ROS board has sent us.
This is how we create a topic and callback function:
```
//For ROS, the name of the topic is gun_control and the ROS board can send 
//messages only of type Int16. The name of the function in Arduino that needs 
//to be called every time we receive a message is shootGun
ros::Subscriber<std_msgs::Int16> gun_ctrl("gun_control", &shootGun);   
``` 

Now we need to implement shootGun:
```
void moveValve(const std_msgs::Int16 & pos_msg){
  
  valve_pos = pos_msg.data;   //Get data sent from ROS
  if(valve_pos == 1){
    ax12a.moveSpeed(ID1, clean_ball_position, servo_speed); //cross-reference library for more info
    }
  else if (valve_pos == 2){
    ax12a.moveSpeed(ID1, dirty_ball_position, servo_speed);
    }
  else if(valve_pos == 3){
    ax12a.moveSpeed(ID1, initial_pos_purifier, servo_speed);
    }
}
```

### Set-up
We need to setup the dynamixel, subscribe to the our topics and we need to initialise the ROS node we created.
``` 
  //Start-up
  ax12a.begin(BaudRate, DirectionPin, &Serial);
  //Remove endless rotation
  ax12a.setEndless(ID1, OFF);
  ax12a.setEndless(ID2, OFF);
  //move into initial position
  ax12a.moveSpeed(ID1, initial_pos_purifier, servo_speed);
  ax12a.moveSpeed(ID2, initial_pos_bee, servo_speed);
  
  nh.initNode();
  
  nh.subscribe(bee_ctrl);
  nh.subscribe(ballseparator_ctrl);
  nh.subscribe(gun_ctrl);
```

### loop
Keep it simple.
```
nh.spinOnce();    //Checks topics that we subscribed to and calls callback function if ROS board has published something correctly. 
```

### What ROS needs to send
To properly implement this code, you will need a ROS board and an Arduino board. The default baudrate is 57600 with ROS. ROS needs to publish:

* true or false to the topic "bee_ctrl" to position dynamixel
* 1, 2 or 3 to the topic "ballseparator_ctrl" to position dynamixel
* 0-255 to set the duty cycle of the PWM signal at pin 9 of the Arduino.


# References

-	e-manual : http://support.robotis.com/en/ 

-	youtube vidéo for all the steps of the use of the Dynamixel Wizard : https://www.youtube.com/watch?v=YJ9b68hx5Qc&version=3&hl=ko_KR

-	website of the manufacturer : http://en.robotis.com

- AX-12A library : https://github.com/ThingType/AX-12A-servo-library


