## If we had to redo it 

>*last updated on May 11, 2018*
> 

If we had to design another robot with technologies described, there are some decision that we would change. 
* PCB Design

The ECAM installation for PCB printing is very useful for prototyping. It is not comparable with PCB that specialised company can create. So, our advice is the following, as soon as a board is functional and definitive, send it to
have a proper and reliable board. Moreover, try to reach this step as soon as possible. 

* Connection 

We have underestimated the connection on the different board. Regarding the JST connector, they are solid but hard to disconnect and cable creation is  long without the right tool. About the 10 pins shrouded header, they are not 
reliable enough. Opposite to the JST, disconnection is too easy. We really  advise against its use. They were the source of a lot of problems. About the SPI connector (10 pins shrouded header) it was not strong enough. The contact
failure brought a lot of problems in the communication between ROS  serial and ATmega.

In the two situations, we advise against the use of these two types of connector, there are a lot of alternative solutions. 

![JST connector](electronics/pcb/pictures/JST_Connector.jpg) 

![10 pins shrouded headers](electronics/pcb/pictures/10Pins_shroudedHeaders.jpg)
