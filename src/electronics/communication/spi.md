# Serial Peripheral Interface (SPI)
>*last updated on May 10, 2018*
> 

## Quick theoretical reminders 

The Serial Peripheral Interface bus (SPI) is a synchronous serial communication interface specification used for short distance communication, primarily in embedded systems. 

You can see on the picture below the topology of this communictaion interface and the pinout of the components:  

![](https://upload.wikimedia.org/wikipedia/commons/thumb/f/fc/SPI_three_slaves.svg/363px-SPI_three_slaves.svg.png)

SPI only defines the physical and data link layer of OSI network model. The connection and media between the devices indicates the physical layer. Data link layer defines the way in which the devices are connected. The connections include the input and output lines such as clock, data in and data out. Data link layer in SPI is implied in the connection itself and no provision is made for location or address information. 

The communiction frame is writen for the use (Layer three on OSI Model). To begin a communication with a *Slave*, the *Master* must fix the Slave Select pin (SS) to **LOW** state. After that, the master can put some bits into the shift register throught the MOSI  (Master Output Slave Input) pin. 
When the communication is done, it's the master who must put back the SS to **HIGH** State.

> **Note** :
> It's a full duplex interface, The slave can push some data on his MISO (Master Input Slave Output) pin during the communication.


![](https://upload.wikimedia.org/wikipedia/commons/thumb/b/bb/SPI_8-bit_circular_transfer.svg/500px-SPI_8-bit_circular_transfer.svg.png)


### advantages 
+ Full duplex communication
+ Flexibility of the number of bits to be transmitted as well as the protocol itself
+ no possible collision

### disadvantage
+ \(3 + N*Slaves\) pins are required on the master
+ 4 pins are required on each slaves
+ Master can speak in a vacuum without knowing it
+ Bus can only count one master
 
## Use of our SPI

the communication frame we used to send data to the slave is:

1. Register adress on 8 bits
2. Number of bytes composing the message
3. Message


> **Note** :
> to read some data from a slave's register, the master must just send the register adress. The slave push the data on his MISO (Master Input Slave Output) pin.

### Our Master 
Create a SPI Master is really easy with a atmega328p and the arduino IDE. Indeed, we can find easily some libraries. We used the one provided directly by the arduino IDE in an object that we implemented : [SPIManager](https://github.com/Ecam-Eurobot/Eurobot-2018/blob/master/arduino/SPIManager.cpp)

how to work with [SPIManager](https://github.com/Ecam-Eurobot/Eurobot-2018/blob/master/arduino/SPIManager.cpp) object will be illustrated at least code use for Cortex's engines control (Eurobot 2018) : [MotorBroker](https://github.com/Ecam-Eurobot/Eurobot-2018/blob/master/arduino/Ecam/examples/MotorControl/MotorBroker/MotorBroker.ino)

##### 1. Initialisation of the SPI communication
The [SPIManager](https://github.com/Ecam-Eurobot/Eurobot-2018/blob/master/arduino/SPIManager.cpp) object must be seen as a communication channel with the slave. It will be necessary to instantiate one [SPIManager](https://github.com/Ecam-Eurobot/Eurobot-2018/blob/master/arduino/SPIManager.cpp) object by slave. When creating the object, use the number of the SS pin as an argument.

```c
//Define the SS pin for each slave
#define FL_SS  9
#define FR_SS  2
#define BL_SS  10
#define BR_SS  3

//Object Instantiation
SPIManager connFL(FL_SS);
SPIManager connFR(FR_SS);
SPIManager connBL(BL_SS);
SPIManager connBR(BR_SS);
```

Use the *initialize()* method to activate the communication channel. This call should be placed in the *setup()* function of the arduino code.

```c
connFL.initialize();
```

##### 2. Write data to a Slave's register
as said above, to write a value in a register of the slave will first send the address of the register, then the size in number of byte and to finish, the message.

The *writedata* function has been written to make its use as intuitive as possible.
below, an example of use where we send to the register **16** (0x10), the message **x.b** whose size is **4 bytes**.

```c
connBR.writeData(0x10, 0x04, x.b);
```

##### 3. Read data from a Slave's register
There are two functions to read some data: the first,*readLongData* , will return a value of type **long** and the other ,*readData* , a value of type **float32**

below, an example of use where we get some data from th register **81** (0x51).

```c
encoders_msg.rear_left = connBL.readLongData(0x51);
```

### Our Slaves
When using the Arduino IDE to program atmeg328p, the slave mode is not available. Two registers must be modified to *activate* this mode :

```c
// turn on SPI in slave mode
SPCR |= _BV(SPE);
// turn on interrupts
SPCR |= _BV(SPIE);
```
So that these manipulations are transparent for the user we also create an object for the SPI bus slaves : [SPISlave](https://github.com/Ecam-Eurobot/Eurobot-2018/blob/master/arduino/SPIslave.cpp)

this object has several methods :

1. **begin()** : change the two registers cited above and declaring the MISO pin as an output. 
2. **reset()** : reset private object variables.
3. **com()**: manage the communication by putting the incoming data in the right variables.

and some properties :

1. **command** : return the register address.
2. **dataSize** : return the size of the incoming message.
3. **msg** : return the message (maximum 64 bits).
4. **endtrans** : boolean that returns **true** if communication is complete.

##### 1. Initialisation of the SPI communication
The MOSI, MISO and SLK pin are always the same on arduino. If you want you can change the SS pin number we agree to use pin 10 as pin SS for the slave.

> **Note** :
> SS is a reserved name used for the SPI communication.


On the begin of our script we create a SPI object and define the SS pin number as it's done in the following code :

```c
SpiSlave mySPI;

//SLK  : pin 13
//MISO : pin 12
//MOSI : pin 11 
#define SS 10
```
to enable the communication, use the *begin()* method of the SPISlave object in the *setup()* function of the atemega :

```c
void setup()
{
	mySPI.begin();
}
```

##### 2. Interruption
An incoming call performed by the master to the slave triggers an interrupt. In most cases, the method in which the interrupt routine will be placed will be: **ISR (name _of _a _register)**. in our case the register will be **SPI _VTC _vect**. As we work whith an atemega328p the shift register is called : **SPDR**.

This way of working is illustrated in the following code :

```c
//Interrupt needed by SPI Communication
ISR (SPI_STC_vect) {
	//manage the communication by putting the incoming data in the right variables.
	mySPI.com(SPDR);
	//Method containing the registers of the Slave. 
	spiReg();   
}
```
##### 3. Registers
A **SWITCH** is commonly used to define the registers. In our case of use, it is the *command* property of the [SPISlave](https://github.com/Ecam-Eurobot/Eurobot-2018/blob/master/arduino/SPIslave.cpp) object which is used as parameter of the **SWITCH**.

the writing in a register will be done once the communication is finished (thanks to the property *endTrans*). This method of work has the advantage of not having inconsistent values during the communication.

The reading of a registers uses the full-duplex channel provided by the SPI. The data are pushed through the channel during the communication using the **SPDR** register.

Here is an example of a function that contains the registers of a slave:

```c
void spiReg(){  
switch (mySPI.command) {
  case 0x10:
      if(mySPI.endTrans) {
          for (int i = 0; i < 4; i++){
              motor_speed.b[i] = mySPI.msg[i];
           }
       }
      break;
   case 0x11:
      SPDR = motor_speed.b[mySPI.dataCount - 2];
      break;
   case 0x50:
      motor_encoder.write(0);
      break;    
   case 0x51:
      SPDR = EncoderState.b[mySPI.dataCount - 2];
      break;    
  } 
}
```

    
### Troubleshooting
>Patched but not fixed

##### 1. Problem between serial port (ROSSERIAL) and SPI bus
There is a competition between Rosserial and the SPI since both use the serial bus. when one ROS sends a data via the serial port, the communication SPI is interrupted.

Patch : simply initialize the SPI communication before each data transfer

```c
void message_to_FR( const std_msgs::Float32 & toggle_msg){
	x.val = toggle_msg.data;
	connFR.initialize();
	connFR.writeData(0x10, 0x04, x.b);
}
```

##### 2. sometimes the slave update the value of a register with the value received previously 
we observed that sometimes the received data are updated with a delay of ONE communication. The problem is that if the time between two communications is long, there will be latency before the update in the registry.

Patch : The data is sent by ROS every 50ms even if it is not changed (see on [motor_sim.py](https://github.com/Ecam-Eurobot/Eurobot-2018/blob/master/ros_packages/mecanum/src/motor_sim.py))


##### 3. Changing the values of a register when reading another register
When we want to read the return of a slave encoder we send a series of null bytes after the address byte to receive the value. But these null values are misinterpreted and the engine speed drops to zero. Probably from a coding error.

Patch : We send velocity value instead of null bytes. (just using during the Eurobot sprint)

# Just for curious.
[https://github.com/Sellto/SPIandROS_HelloWord](https://github.com/Sellto/SPIandROS_HelloWord)

Three simples scripts to train you with ours SPI objects and with or without rosserial communication.

Decription of the situation : 

+ The two slaves has a LED connected to pin 4 whose state is modifiable by a SPI register.
+ The master is connected to ROS which controls the flashing speed of each led.

Video-Tuto : ASAP

# References
 [https://en.wikipedia.org/wiki/Serial_Peripheral_Interface_Bus](https://en.wikipedia.org/wiki/Serial_Peripheral_Interface_Bus)
