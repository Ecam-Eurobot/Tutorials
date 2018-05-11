# The big robot specifications
![alt text](mechanical/2018/bigrobot_render.PNG)
## Fusion 360

The entire robot is designed in Fusion 360 and is available on the Eurobot cloud. The final version is called **SHARK FINAL V17**

Here is the link to download de fusion projet : [BigRobotModel](https://a360.co/2wA0er0)

## The goal of the 2018 edition

![alt text](mechanical/2018/bigRobotSrc/plan_arene.PNG)

The robot starts from the inital surface (**number 1**).

The robot has to collect the cubes and to build a tower of 5 cubes maximum in the construction zone (**number 4**).
The team earns a lot a point if it respects the construction plan shown on the wall (**number 7**).




## Sections ( How does it work ?)

This paragraph explains the differents conceptions's steps and all the parts needed to build a tower. Here is the general flow diagram :

![alt text](mechanical/2018/bigRobotSrc/flow_diagram.PNG)


The system doesn't need any color detection because all the stars of cubes are always in the same order in the same direction.


### The base

![alt text](mechanical/2018/bigRobotSrc/base_render.jpg)

This is the firt thing I'm going to talk about but it was the last designed part. Indeed, every part had to be designed to know the constraints and dimensions. Thoses informations where essentials to design the base.

After the design in Fusion 360 in the model environment, the design was imported to the CAM environment still in Fusion.


![alt text](mechanical/2018/bigRobotSrc/base_cam.PNG)

It was then machined with a CNC :

![alt text](mechanical/2018/bigRobotSrc/base_machining.jpg)

### The entry

To swallow the cubes, an entry was designed. It uses simple DC motor with paint roller bought at the *Brico*.

![alt text](mechanical/2018/bigRobotSrc/entry.PNG)

The entry swallows the first 3 cubes and then select the fourth cube between the 2 last by pushing it with the toothed rack. After that the robot do the same step with the last cube. The rack moves with a rail available on *RS component* :


**IGUS rail**:
 * *Rail* :Igus N Series, NS-01-17-300, Linear Guide Rail 17mm width 300mm length
 * *Carriage* : Igus Linear Guide Carriage NW-02-17, N

To detect when the cube is in the lift, the robot uses a limit switch available on *RS Component* :

**Limit switch** : Snap Action Limit Switch, Roller Lever, Thermoplastic, NC, 125V.

### The lift

The limit switch used in the entry is connected to a pin of an Arduino Uno board so we can detect the state change when a block touches the switch. 
That event triggers the lift and thus the block starts to raise up. 
Therefore we are using another DC motor with a belt as you can see on the picture below. 

![img](mechanical/2018/bigRobotSrc/lift_motor_belt.png)

Thanks to the colour detection we know in which order we have to collect the blocks on the field, so we also know which one comes first in the lift and which floor it has to go to. 
It is important to know the specific floor in order to create the pattern showed at the beginning of the game and recognised by the colour and sequence recognition part.

To know when the lift has to stop, we use the encoder of the DC motor so we can precisely know when the motor has to stop. 
The encoder is connected to an interruption pin on the Arduino board so the tick counts can be more accurate and have priority.

Once the block is on the right floor, it has to be pushed outside of the lift to already be placed in its position on the tower on a kind of shelf (as you can see in the picture below) which supports the blocks even if there is no other block below. 

![img](mechanical/2018/bigRobotSrc/cage.jpg)

Furthermore, to push the block, we use a dynamixel with a gear and a tooth rack to slide a plate along which pushes the block until it reaches the « shelf ». 

![img](mechanical/2018/bigRobotSrc/tooth_rack.png)

Then the lift (without any block inside) goes down. To make sure it doesn’t go too much we use another limit switch. 
Therefore the lift is immediately stopped, another cube comes in and we can do the cube lifting steps mentioned before over again until all the cubes of the tower are placed. 


As mentioned above, we are using DC motors to control our lift. 
We have in all 3 motors and they are all connected to a L298 driver (see picture below) controlled by an Arduino Uno board.  

Here is a link where you can buy the driver :  

<https://www.amazon.fr/L298-Motor-Driver/s?ie=UTF8&page=1&rh=i%3Aaps%2Ck%3AL298%20Motor%20Driver>

![img](mechanical/2018/bigRobotSrc/driver_L298.png)

We use 2 motors for the rollers to collect the blocks. In order to do so, each one has to turn in the opposite direction of the other but at the same speed. 
Thus we connected them to the same driver but inverted the pins of one motor compared to the other one. 
These motors don’t have an encoder so their only 2 pins are connected to the driver. We also have the third motor to make the lift go up and down. As we said before, we use the encoder to know where we have to stop so there are pins of the motor connected to the Arduino board. On the other hand the motor is also connected to the driver so we can control its speed, stop it and launch it.


### The cage

![alt text](mechanical/2018/bigRobotSrc/cage.jpg)

The floors can rotate thanks to a dynamixel and a bearing wheel. To place the bearing wheel on the rotating axis we insert the last floor into the others floors with the bearing wheel between them as shown in the next figure :

![alt text](mechanical/2018/bigRobotSrc/cage_assembly.PNG)


## External parts

Most of the part are home made (CNC, 3D printing) but some parts a bought on the market :

* **IGUS rail**
  * *Small one*
    * Rail : Igus N Series, NS-01-17-300, Linear Guide Rail 17mm width 300mm length
    * Carriage : Igus Linear Guide Carriage NW-02-17, N
  * *Big one* : 
    * Rail : Igus N Series, NS-01-27-300, Linear Guide Rail 27mm width 300mm length
    * Carriage : Igus Linear Guide Carriage NW-02-27, N
* **Lift transmission**
  * *Pitch (distance between teeth)* : 2.032 mm
  * *Pulley* : aluminium, Glass Filled PC Timing Belt Pulley, 6mm Belt Width x 2.032mm Pitch, 36 Tooth, Maximum Bore Dia. 5mm
  * *Timing belt* : RS Pro, Timing Belt, 315 Tooth, 640.08mm Length X 6mm Width

* **Limit switch**
  * Snap Action Limit Switch, Roller Lever, Thermoplastic, NC, 125V
  
  
  # Authors
  
  Crappe Martin  
  Hagopian Armen
  


