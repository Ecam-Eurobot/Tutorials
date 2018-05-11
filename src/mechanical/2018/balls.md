# The Cortex (small) Robot

![alt text](mechanical/2018/BallGun_SRC/Cortex.png ) 

This year, the Eurobot task was water purification. As water is not practical around prototype electronics, they used balls to represent water. The balls of the
same colour that is assigned to you during a match will represent clean/purified water whereas balls of the same colour as your opponents represent dirty water
that has to be treated. To ease readability the word "balls" will be used synonymously with "water" for the rest of this article. We are also going to assume that
the team colour assigned to our team is GREEN and therefore green balls = clean water and orange balls (the colour of the opponent team) = dirty water.

At the start of each match the Cortex robot has to collect 8 balls from two different containers on the playing field, sort them and then send them either to the
"home tank" if they are green balls and into the reservoir if they are orange balls. We will take a look the mechanisms we put in place to collect, sort and 
send balls during each match for Eurobot 2018.

# Ball Mechanism

![alt text](mechanical/2018/BallGun_SRC/BallSeparationMech.png )

## Introduction

Initially, after much thought, we created a system composed of 3 basic parts and a colour detector. There are two parts that are mobile parts and one stagnant outer
case. The system is divided into 4 compartments: the entry point, the colour sensor (CS) compartment, green trap compartment and the orange trap compartment.
The colour detector is embedded in the stagnant outer case. Underneath the outer case is a moving plate controlled by an actuator (the dynamixel AX-12A) and a
rotating "pusher" controlled by a stepper motor to move balls within the outer case.

Essentially there are 4 main steps for a ball to be separated in this system:

* The ball enters the mechanism at the entry point.
* The stepper motor activates and moves the ball to the CS compartment.
* The colour sensor detects the colour of the ball. Once  the colour of the ball is known, the dynamixel AX-12A is activated and the opening of the mobile plate
is moved into the corresponding colour position.
* The stepper motor activates and moves the ball into the green trap or orange trap.

The interest behind this idea is the fact that as the "pusher" is activated and moving another ball can enter at at the entry point and therefore creating a chain
effect which we hoped would gain us time. 


## The outer case and colour sensor

![alt text](mechanical/2018/BallGun_SRC/OuterCase.png )

As you can see here we have the entry point and the colour sensor. How the colour sensor works and its corresponding code can be found in the sensors section.

## The mobile plate

![alt text](mechanical/2018/BallGun_SRC/purifier.png )

Control of the mobile plate was done with a dynamixel AX-12A. 

## The "pusher"

![alt text](mechanical/2018/BallGun_SRC/Pusher.png )

The pusher as well as the colour detector were eventually excluded from initial prototyping for the Cortex robot because of time constraints and on the basis 
of a strategic decision to simplify the mechanism for the qualifying stages. However, the link to an open-source library very useful for controlling stepper motors 
is supplied in this document under the actuator section as well.

## Ball mechanism used in qualifying stages

![alt text](mechanical/2018/BallGun_SRC/Separation.png )

This is the system that we used during qualifying stages which only makes use of a mobile plate and rounded edges on the sides and corners
to guide the balls collected. The mobile plate was controlled by dynamixel and the corresponding code and application can be found under the 
actuators section of this report.

# Ball Gun (Sending Mechanism)

![alt text](mechanical/2018/BallGun_SRC/BallGun3D.png )

## 3D sketch

All the parts of the ball gun is designed on Fusion 360 and printed on a 3D printer except of the balls and the wheel.

## Why a ball gun?

For the 2018 contest, we had to collect and sort balls by color and then put all the balls collected in the color of the team previously known in a big tank that made reference of a water castle as the balls made reference of water itself. The more balls we threw in the tank, the more points we earned.

We choose to throw the balls with a gun made of a ramp for the direction of the ball and a wheel with a DC motor. as shown on the picture, you can see a fixed ramp, an adjustable ramp and the wheel.

## Construction step by step

After lots of discussions with the team, I decided to choose the ball gun to complete the task of throwing balls in the tank. So I first made a prototype of the ball gun made of wood and a small DC motor I found in the lab but for the first tests it appeared that the small DC motor wasn't powerfull enough. So I choose a more powerfull and a better speed motor as a Maxon motor found in the lab and the results were very conclusive. 

![alt text](mechanical/2018/BallGun_SRC/BallGunPrototype.jpg )

The next step was to control the speed of the wheel so we could control the distance the ball make after throwing by the ball gun.

I used a small motor driver as the L293D which the specifications were in agreement with the needs of the DC motor. 

![alt text](mechanical/2018/BallGun_SRC/L293DAbsolute.png )

![alt text](mechanical/2018/BallGun_SRC/L293DRecommended.png )

You can find the datasheet of the L293D here : https://github.com/Ecam-Eurobot/Tutorials/tree/StevenGaro-patch-1/src/mechanical/2018/BallGun_SRC

To control the H bridge driver, I used a simple arduino uno with a PWM signal so we could test different speeds of the motor. 

![alt text](mechanical/2018/BallGun_SRC/SchemaBlock.png )

The precision of the distance the ball make was made experimentaly when we did the different tests.

The code I first used to test the driver with the motor is the next code in arduino :

```
int inputPin = A0;  // set input pin for the potentiometer
int inputValue = 0; // potentiometer input variable

const int motorPin1  = 5;  // Pin 14 of L293
const int motorPin2  = 6;  // Pin 10 of L293

void setup() {
     // declare the ledPin as an OUTPUT:
     pinMode(motorPin1, OUTPUT);
     
}

void loop() {
     // read the value from the potentiometer:
     inputValue = analogRead(inputPin);

     // send the square wave signal to the LED:
     analogWrite(motorPin1, inputValue);

  digitalWrite(motorPin1, HIGH);
  delayMicroseconds(inputValue); // Approximately 10% duty cycle @ 1KHz
  digitalWrite(motorPin1, LOW);
  delayMicroseconds(1023 - inputValue);
     
}
```
When the tests were conclusive enough, we designed a ramp that fit in the robot correctly with an adjustable ramp for the ball to go higher or lower in case we need to modify the distance during the competition.

First we made the fixed ramp that comes from the ball sorting mecanism and end around the wheel.

![alt text](mechanical/2018/BallGun_SRC/FixedRamp.png )

After that we made a straight ramp that shows the ball the direction it must take.

![alt text](mechanical/2018/BallGun_SRC/StraightRamp.png )

And finaly, we designed a small piece that maintain the straight ramp from below and that you can adjust.

![alt text](mechanical/2018/BallGun_SRC/PieceAdjust.png )

After that, all we had to do is to install the gun in the robot and make some tests to determine the best power to give as a PWM to the motor to throw the balls from the distance chosen in the tank and to incorporate the code of the ball gun in the main code of the robot.


