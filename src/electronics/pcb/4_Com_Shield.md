## Arduino Communication Shield - SPI Broker

>*last updated on May 11, 2018*
> 

Motor command communication consists of a two-level interface. ROS communicate via ROS Serial (explained in the appropriate section) implemented on an 
arduino UNO and this arduino handle the communication with the four individual motors. To help for connections, we designed an arduino shield gathering
simply all SPI bus connections. The only role of the PCB is deleting redundancy in the SPI bus connections and consequently facilitating the SPI bus 
execution. As the PCB is really simple, it implements a bus SPI connection, it requires no explanation. 

The utility of this PCB concerns SPI and Ros Serial, you will find more explanation about these subjects in the related section. 

![Broker Front View](electronics/pcb/pictures/Broker_F.jpg)

![Broker Top View](electronics/pcb/pictures/Broker_T.jpg)

--- [OneDrive link for Altium project](https://) 
