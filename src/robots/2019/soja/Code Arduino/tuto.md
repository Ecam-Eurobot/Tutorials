# Read the value of one encoder
This code allows to show the value return by one encoder.

* Connect the Arduino to the PC
* Download the code on GitHub named “OneEncoderUno.ino” 
* Compile and upload the code (verify that you select the right model and COM) 
* Open the serial monitor
* Turn the encoder and see the value of the rotation

# Test if the motors are working 

This code allows to test only one motor. 

* Connect the Arduino to the PC
* Download the code on GitHub named “TestOneMotor.ino” 
* Connect the motor that you want to test with the plug named Motor 2
* Connect the H-Bridge to the Arduino:  
  * GND on pin GND
  * PWM1 on pin 9
  * DIR1 on pin 11
  * 5V on pin 5V	
* Connect the inputs GND and POWER of the H-Bridge to an alimentation of 12V or to power board. 
* Compile and upload the code (verify that you select the right model and COM) 
* The motor will turn if everything is OK. 


# Test the two encoders

This code will show you the value return by two encoders, for this code use the shield. 

* Connect the Arduino to the PC
* Download the code on GitHub named “TwoEncodersOnly.ino” 
* Place the shield on the Arduino
* Plug the two encoders in the right place 
* Compile and upload the code (verify that you select the right model and COM) 
* Turn the encoders and see the value on the serial monitor



# Test the two encoders
This code will show you the value return by two encoders when the robot moves, for this code use the shield. 

Caution: The robot will not go straight ahead because we give the same speed on the two motors. 

* Connect the Arduino to the PC
* Download the code on GitHub named “TwoEncodersAndPMega.ino” 
* Place the shield on the Arduino
* Plug the two encoders in the right place 
* Connect the shield with the H-bridge
* Connect the motors to the H-bridge (Right Motor --> Motor 1 & Left Motor --> Motor 2)
* Connect the H-bridge to the Power board or to a power source
* Modify the value of the setpoint f-if you want
* Compile and upload the code (verify that you select the right model and COM) 
* See the robot decelerate when he approaches the value of the setpoint


# Test the displacement of the robot

* Connect the Arduino to the PC
* Download the code on GitHub named “NavigSojaPosAct.ino” 
* Place the shield on the Arduino
* Plug the two encoders in the right place 
* Connect the shield with the H-bridge
* Connect the motors to the H-bridge (Right Motor --> Motor 1 & Left Motor --> Motor 2)
* Connect the H-bridge to the Power board or to a power source
* Write on the void loop the movement that you want
  * Linear (distance in meter) or Rotation (angle in degree)  
  * To go backward put a minus before the distance
  * Positive angle = rotation to the right and negative angle = rotation to the left
* Compile and upload the code (verify that you select the right model and COM) 




