author : Emine Tas  
date : 03/05/2019

# Servomotor programming
## Metal rods

### What's a servomotor ?
*" It's a motor capable of maintaining an opposition to a static force and whose position is continuously checked and corrected according to the measurement. " Wikipedia*  
*" The position is defined with an angle travel limit of 180 degrees, but also available in continuous rotation. " Wikipedia*  
 It have an angle generally ranging from 0 to 180 °.

### How to connect it?
It's composed of 3 cables : red, brown and orange.   
* Red for + (5V)
* Brown for - (ground)
* Orange for signal


### What is his purpose in this project ?
Simply rotate continuously from right to left or from left to right depending on
the position of the robot in the field.  


```C
/* The "Servo" library must be included to handle the servomotor */
#include <Servo.h>

/* Creating a servo object to control the servo motor */
Servo monServomoteur;
Servo monServomoteur2;

void setup() {
  /* Attach the servomotor to pin x */
  monServomoteur.attach(9);
  monServomoteur2.attach(10);

  /* According to the signal on the pin 7 activation of one of the servomotor */
  /* Definition of the starting point at 90° */
  if(digitalRead(7)){
     monServomoteur2.write(90);
     delay(15);
  }else{
     monServomoteur.write(90);
     delay(15);
  }
}


void loop() {
  /* The left one : right to left */
  if(digitalRead(7)){
    for (int position = 110; position <= 40; position++) {
      monServomoteur2.write(position);
      delay(15);
    }
  }else{
  /* The right one : left to right */
    for (int position = 110; position >= 40; position--) {
      monServomoteur.write(position);
      delay(15);
    }
  }
}
```


### Tips for construction
If you want to make a assembly with screws caution !
To screw the rod on the servomotor it was necessary to make a big hole so that it does not turn in void.
