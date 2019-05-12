# Motors
This section is used to explain the code used on the Arduino that is used to drive the motors of the big robot and provide feedback of its movements to the raspberry pi.
We think this code could be reused with minimal changes the following years as it solves very basic problems that transcend the competition rules.

You can find this code on the 2019 github repository at the following address: [https://github.com/Ecam-Eurobot/Eurobot-2019/blob/grand_robot/arduino/motors.ino](https://github.com/Ecam-Eurobot/Eurobot-2019/blob/grand_robot/arduino/motors.ino)

| Don't make the mistakes that we did and try to reuse as much code as you can from previous years! |
| --- |

The hadware environment in whitch this code is supposed to work is described in [the following chapter](robots/2019/soja.md)

That being said let's take a look at what the code does.

## Main purposes
This code has 3 main purposes:
    1. Receive driving instructions over ROS and translate them to signals to send to the motors
    2. Send encoder data back over ROS so we can localize the robot on the map
    3. Activate the mechanical arms that we installed for this year's challenge

We also included some debugging functionalities to test the encoders using LEDs for example. But let's not get ahead of ourselves.

First of all, we have to include all the used libraries. This includes the ROS library to subscribe to ROS topic; the servo library is used to control the mechanical arms; a PID library will manage the PID control loop to drive in a straight line and turn around the center of our robot; and finally, various message types that will be used to transfer information over ROS.
```arduino
#include <FastPID.h>
#include <Arduino.h>
#include <Servo.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3.h>
```

Once this is done, all our used pins and variables have to be declared and defined. We tried to keep the names as self-explanatory as possible. As such, `#define c_LeftEncoderPinA 2` means that the first of the two outputs of the left wheel encoder is connected to pin 2 of the Arduino. `c_RightMotorPwmPin` will be mapped to the pin that will transmit the pulse-width modulated signal that will define the speed of the right motor.

```arduino
void setup() {
    Serial.begin(57600);

    // init ROS
    nh.initNode();
    nh.subscribe(sub_lin);
    nh.subscribe(sub_rot);
    nh.subscribe(sub_arm);
    nh.subscribe(sub_stop);
    nh.advertise(pub_encoder);
    nh.advertise(pub_feedback);

    pinMode(c_LeftEncoderPinA, INPUT);
    digitalWrite(c_LeftEncoderPinA, LOW);
    pinMode(c_LeftEncoderPinB, INPUT);
    digitalWrite(c_LeftEncoderPinB, LOW);
    attachInterrupt(digitalPinToInterrupt(c_LeftEncoderPinA), HandleLeftMotorInterrupt, CHANGE);

    pinMode(c_RightEncoderPinA, INPUT);
    digitalWrite(c_RightEncoderPinA, LOW);
    pinMode(c_RightEncoderPinB, INPUT);
    digitalWrite(c_RightEncoderPinB, LOW);
    attachInterrupt(digitalPinToInterrupt(c_RightEncoderPinA), HandleRightMotorInterrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(c_RightEncoderPinB), HandleRightMotorInterrupt, CHANGE);

    pinMode(c_LeftLedPin, OUTPUT);
    digitalWrite(c_LeftLedPin, LOW);
    pinMode(c_RightLedPin, OUTPUT);
    digitalWrite(c_RightLedPin, LOW);

    pinMode(c_LeftMotorPwmPin, OUTPUT);
    pinMode(c_LeftMotorDirPin, OUTPUT);
    pinMode(c_RightMotorPwmPin, OUTPUT);
    pinMode(c_RightMotorDirPin, OUTPUT);

    if(diff_PID.err()) {
        nh.logerror("There is a configuration error!");
        for (;;) {}
    }

    TCCR2B = TCCR2B & 0b11111000 | 0x01;

    right_arm.attach(c_RightServoPin);
    left_arm.attach(c_RightServoPin);
}
```

Several variables will contain specific parameters to fine tune the PID loop and others will be used to store useful information such as the number of ticks one encoder has traveled or the messages that will be send over ROS.

After that, we will instanciate certain objects, provided to us by the libraries we included, so we can make use of them later.

We now have to declare our callback functions that are executed every time nh.spinOnce() is called if a new message is received on a ROS topic. "What?!" I hear you say, "We haven't even started ROS or even told it on which topics to listen!". And you'd be right. But we have to declare those functions before we tell ROS what topics to subscribe to because in the same line we also have to point to those callback functions that we are declaring. This is done with a pointer which has to point to an already existing function. So that's why we have to declare the function before we initiate ROS. Pfew! So here we are declaring those functions. Those are `lin_cb`, `rot_cb`, `arm_cb`, and `stop_cb`, who respectively listen to the `cmd_lin`, `cmd_rot`, `cmd_arm`, and `cmd_stop` topics. Each of these functions do similar things: first, they expect some object in input. These objects are given to us by ROS and will contain the last message received on the topic. Inside the function we then "unpack" the values from those message objects by assigning their attributes to variables declared earlier. Next, we call an external function that will execute an bction based on what message was received on what topic. Those functions will be written later in the file.

We now arrive at the infamous `void setup()` function provided by the Arduino. As you all certainly know, the Arduino will execute the code inside this function once right after every boot. In this section we will begin the serial communication needed to communicate over ROS (with a baud rate of 57600 to limit the latency of the channel), initiate ROS, actually subscribe to the topics declared earlier, declare all the in and outputs, as well as this peculiar line:

```arduino
TCCR2B = TCCR2B & 0b11111000 | 0x01;
```
What this does is change the Arduino Mega's pwm frequency on pin 9 and 10 to 31.37255kHz instead of the default 490.20Hz. You can find various explanation on the Arduino forum, but the only official Arduino documentation page that somewhat explains this feature is [this one](ttps://www.arduino.cc/en/Tutorial/SecretsOfArduinoPWM). 

After this the only other function that is needed for our program to be compilable is `void loop()`. The code in this loop is actually nothing else than an infinite `while(true)` loop that the Arduino will keep running over and over again until the end of times. You can see this loop is much smaller than the previous one.

```arduino
void loop() {
    encoder_msg.x = _LeftEncoderTicks;
    encoder_msg.y = _RightEncoderTicks;
    pub_encoder.publish(&encoder_msg);

    nh.spinOnce();
}
```
This is pretty straightforward. We set the `x` and `y` attributes of the `encoder_msg` object (whose type is geometry\_msgs/Vector3. The `z` attribute of that oject is not used because we are working on a 2D plane) to the number of ticks the left and right encoder have traveled respectfully. We then have to tell ROS to puplish `encoder_msg` on the `encoder_ticks` topic the next time nh.spinOnce() is executed, which happens right after. Optionally, we can also turn our LEDs on or off for debugging purposes here.

We now come to the final section of our code, which is where we declare all the functions that actually tell the motors to turn a certain way or count the number of impulses each encoder is giving in real time.
I won't explain the insides of all those functions here, as they could almost all use a seperate documentation on their own.

The essensial thing here is that those functions are not *too* badly written and work well as they are. There should be no reason to modify them unless there is a change in harware.
