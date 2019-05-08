
>*last updated on May 11, 2018*
> 

## Motor Board

The goal of this board is to bring individual intelligence for every motor used on our robots. It allows to associate a microcontroller (ATmega328) with each motor. This main component is used for two aspects : regulation and communication. 

### Regulation

For the smaller robot, the operating system send speed consign to the four motors. Consequently, the goal for the motor is reaching this speed in a minimum of time and after conserving this value. The regulator implemented in the microcontrollers is Proportional Integral Derivative regulators. It's parameters allow to influence the way to consign reaching for the motor. Thanks to it, each motor, individually can follow the ROS order with a maximum of precision. 

PID's parameters were found with experimentation and all sources used for implementation are in the code file. 

### Communication

As said in the previous part, this is the operating system ROS the main brain of the robot. Consequently, this brain has to communicate with its slave.
The communication protocol used on our robots is SPI. It induces that a bus has to be built between all motor and ROS. Effectively, a broker gathers 
the signal for the motor but in any case each motor has to be linked to the bus. A role of the board is this connection via a SPI connector.

### Getting Started
To simplify the tutorial, we will use an arduino UNO instead of ATmega328 microcontroller. For the robot, the reason why we use a ATmega328 alone is the size of the system. The microcontroller and its oscillator are sufficient for our application and take less place on the robot. However, in this tutorial, we will use the arduino UNO to slightly simplify manipulation.

Regarding the regulation, the following code implements the PID regulator. Its parameters were found after experimentation. It depends on the motor and the load. We simply adjusted the parameters until the motor behaviour was the expected one. (mr. Marchand knows the methodology ;) )

The blocks of the system are described below :

* Encoder

The encoder sends the rotation information to the controller. In our code, we used the _Encoder.h_ library to handle it. At this moment, we have the velocity of the wheel in rad/s.

* Motor control

The H bridge is used for power two motors at the same. One the left side of the board there are all control pins (PWM and DIR for the two motors) and power supply of command (5V). On the other side of the board, you have the motor output and the motor power supply (24V).

```cpp
#includes <Wire.h>
#includes <PID_v1.h>
#includes <ecamlib.h>

#include <Encoder.h>
#include <FlexiTimer2.h>

// Defines the pins to control the motor driver
// pin 5 controls the voltage to the motor by a PWM
// pin 4 controls the direction of the motor
const int PWM = 5;
const int DIR = 4;

// Define pin and tick per revolution of the encoder
Encoder motor_encoder = Encoder(2, 3);
const int ENCODER_TICKS_PER_REV = 3200;

// Defines the time between two samples in milliseconds for speed capture
const int CADENCE_MS = 50;
volatile double dt = CADENCE_MS / 1000.;

// Motor control variables and PID configuration
float motor_speed = 6.28319;
float gain_p = 15.0;
float gain_i = 5.0;
float gain_d = 0.5;

// Define angular velocity
volatile double omega;

// PID declaration function of used library
double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint,gain_p,gain_i,gain_d, DIRECT);

// Variable to keep track of the old encoder value
volatile long old_encoder = 0;

void setup() {
    // set timer 0 divisor to 1 for PWM frequency of 62500.00 Hz
    TCCR0B = TCCR0B & B11111000 | B00000001;

    pinMode(PWM,OUTPUT); 
    pinMode(DIR,OUTPUT);
 
 // Periodic execution of isrt() function thanks to FlexiTimer2 library    
    FlexiTimer2::set(CADENCE_MS, 1/1000., isrt); 
    FlexiTimer2::start();

    myPID.SetMode(AUTOMATIC);
    myPID.SetSampleTime(CADENCE_MS);

    // Motor direction 
    digitalWrite(DIR,LOW);
        
}

void loop() {
 
 Setpoint = (double) (abs(motor_speed));
    if (motor_speed > 0) {
        digitalWrite(DIR, LOW);
    } else {
        digitalWrite(DIR, HIGH);
    }
    
    Input = abs(omega);
    
    myPID.Compute();
    
    analogWrite(PWM, Output);
    
    Serial.print("Omega : ");
    Serial.println(omega);
    delay(CADENCE_MS);
}

// Speed measurement 
void isrt(){
    int deltaEncoder = motor_encoder.read() - old_encoder;
    old_encoder = motor_encoder.read();
    
    // Angular velocity 
    omega = ( (2.0 * 3.141592 * (double)deltaEncoder) / ENCODER_TICKS_PER_REV ) / dt;  // rad/s
}
```

Remark : All used library is on Github (see link at the end)


* Controller

Motor are controlled by the arduino with PWM signals. The implemented PID regulator computes the necessary duty cycle to send for reaching the target velocity function of its parameters. 

The following complete circuit is shown below

--- PIC OF THE CIRCUIT --- UNO + H Bridge + MOTOR - DRAW CONNECTION --- After robot's deconstruction

The implemented PID regulator computes to reach the target velocity. So at this moment, you have an arduino UNO controlling motor in velocity thanks to a PID regulation. All regulation is visible in the Serial Window of the Arduino software. 

Now the next step is to give the control of the motor velocity to ROS core on the raspberry. For that, we implemented a SPI connection with ROS. These concepts are explained in the ROS serial section. On the motor side, a function is added on the main code on the arduino UNO to receive data from master and set value of PID (proportional gain, integrative gain, derivative gain and velocity setpoints.

```cpp
// function that executes whenever data is received from the master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) {
    if (Wire.available() >= 2) {
        unsigned char reg = Wire.read();

        switch (reg) {
            // Configuration
            case 0x00:
                Serial.println("Tried to change the configuration, but it's not handled yet...");
                break;

            // Set motor speed
            case 0x10:
                // Expecting a 4 byte float value representing
                // angular velocity
                if (Wire.available() >= 4) {
                    unsigned char b0 = Wire.read();
                    unsigned char b1 = Wire.read();
                    unsigned char b2 = Wire.read();
                    unsigned char b3 = Wire.read();
                    motor_speed = bytesToFloat(b0, b1, b2, b3);
                    Serial.print("Motor speed set to ");
                    Serial.print(motor_speed, 3);
                    Serial.println(" rad/s");
                }
                break;

            // Set PID gains
            case 0x20:
                // Expecting a 4 byte float value representing
                // proportional gain for the PID
                if (Wire.available() >= 4) {
                    unsigned char b0 = Wire.read();
                    unsigned char b1 = Wire.read();
                    unsigned char b2 = Wire.read();
                    unsigned char b3 = Wire.read();
                    gain_p = bytesToFloat(b0, b1, b2, b3);
                    Serial.print("Proportional gain set to ");
                    Serial.println(gain_p, 3);
                }
                break;
            case 0x21:
                // Expecting a 4 byte float value representing
                // integral gain for the PID
                if (Wire.available() >= 4) {
                    unsigned char b0 = Wire.read();
                    unsigned char b1 = Wire.read();
                    unsigned char b2 = Wire.read();
                    unsigned char b3 = Wire.read();
                    gain_i = bytesToFloat(b0, b1, b2, b3);
                    Serial.print("Integral gain set to ");
                    Serial.println(gain_i, 3);
                }
                break;
            case 0x22:
                // Expecting a 4 byte float value representing
                // derivative gain for the PID
                if (Wire.available() >= 4) {
                    unsigned char b0 = Wire.read();
                    unsigned char b1 = Wire.read();
                    unsigned char b2 = Wire.read();
                    unsigned char b3 = Wire.read();
                    gain_d = bytesToFloat(b0, b1, b2, b3);
                    Serial.print("Derivative gain set to ");
                    Serial.println(gain_d, 3);
                }
                break;
            
            default:
                Serial.println("Unexcpected register access over I2C");
                break;
        }
    }
}
```

### Upload protocol for ATmega 328 
If you understood the tutorial with arduino UNO and want to use the ATmega328, you just have to use PCB with connection show below and implement the code exposed above on the ATmega328. This mocrocontroller is programmed with the ISP Programmer explains in the relative section.

--- PIC OF THE CIRCUIT --- Motor board + H Bridge + MOTOR - DRAW CONNECTION --- After robot's deconstruction

[Github link for code](https://github.com/Ecam-Eurobot/Eurobot-2018/tree/arduino-pid/arduino)

--- [OneDrive link for Altium project](https://) 

