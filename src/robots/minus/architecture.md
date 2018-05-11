# Architecture

![Architecture](/img/robots/architecture.png)

There is a raspberry master who talks to the arduinos via the serial port thanks to the `rosserial` library.

The arduinos who control the motors return the values of the encoders to the rapsberry and the raspberry returns them the speeds to apply.

An arduino takes care of the control of the elevator, the collection of the blocks and sends the values of the ultrasound sensors to the raspberry.