# The colour Sensor

There are a wide range of colour sensors currently available today. After studying various sensors that are currently on the market we 
decided to use the RGB Colour Sensor with IR filter and White LED from Adafruit mostly because of its size, cross-platform support (including python code https://learn.adafruit.com/adafruit-color-sensors/circuitpython-code) 
and the fact that it communicates with I2C, the communication bus we had planned on using initially. Eventually we bought the even smaller version, the Flora Colour Sensor with White Illumination LED,
both of them use the same integrated circuit, the Adafruit TCS34725.

On board circuitry includes: 
* 3.3V regulator so you can power the breakout with 3-5VDC safely and level shifting for the I2C pins so they can be used with 3.3V or 5V logic
* neutral 4150°K temperature LED with a MOSFET driver on board to illuminate what you're trying to sense. 
* An IR blocking filter, localized to the color sensing photodiodes, minimizes the IR spectral component of the incoming light and allows colour measurements to be made more 
accurately

![alt text](electronics/sensors/ColorSensor/Flora.png )


More information on this colour sensor can be found here

## Assembly

### On the Flora:

3.3v -> 3v (red wire)
GND -> GND (black wire)
SDA -> SDA (white wire) 
SCL -> SCL (green wire)

### On the Arduino:
Connect SCL    to analog 5
Connect SDA    to analog 4
Connect VDD    to 3.3V DC
Connect GROUND to common ground

## The library
To start working with this sensor it is advised to start by downloading the Arduino library from the github link provided in the references and install
it into the library folder of your Arduino IDE. Hint: the right Arduino folder is usually found in My Documents. The library is an object-orientated library
that allows to create a colour sensor object and access all  its capabilities through its methods.

## The Code
After installing the library, the best thing to do is start with the examples already provided by the library. I started with and tweaked the mostly the 
Colorview code because it allows to give us values between 0 and 255 for the different RGB colours (not 100% accurate but accurate enough) of the 
object being detected. here's an example:

```cpp
#include <Wire.h>                  //include Wire.h to be able to communicate through I2C on Arduino board
#include "Adafruit_TCS34725.h"     //Colour sensor library

//Create colour sensor object declaration, to see effects of different integration time and gain
//settings, check the datatsheet of the Adafruit TCS34725.  
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

void setup() {
  Serial.begin(9600);
  Serial.println("Color View Test!");

  //Start-up colour sensor
  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1); // halt!
  }
}


void loop() {

  uint16_t clear, red, green, blue;
  uint16_t upperrangeG[3] = {100, 153, 71};
  uint16_t lowerrangeG[3] = {50, 104, 52};
  uint16_t upperrangeO[3] = {216,75,44};
  uint16_t lowerrangeO[3] = {155, 60, 32};  

  //Collect raw data from integrated circuit using interrupts
  tcs.setInterrupt(false);      // turn on LED

  delay(60);  // takes 50ms to read 
  
  tcs.getRawData(&red, &green, &blue, &clear);
  tcs.setInterrupt(true);  // turn off LED

  // Figure out some basic hex code for visualization
  uint32_t sum = clear;
  float r, g, b;
  r = red; 
  r /= sum;
  g = green; 
  g /= sum;
  b = blue; 
  b /= sum;
  r *= 256; g *= 256; b *= 256;
  Serial.print("\t");
  
  Serial.print((int)r, HEX); Serial.print((int)g, HEX); Serial.print((int)b, HEX);
  Serial.println();

  Serial.print((int)r ); Serial.print(" "); Serial.print((int)g);Serial.print(" ");  Serial.println((int)b );

  if((r < upperrangeG[0] && g < upperrangeG[1] && b < upperrangeG[2]) || (r > lowerrangeG[0] && g > lowerrangeG[1] && b > lowerrangeG[2])){
    delay(50);
    Serial.print("Green");
  }
  if((r < upperrangeO[0] && g < upperrangeO[1] && b < upperrangeO[2]) || (r > lowerrangeO[0] && g > lowerrangeO[1] && b > lowerrangeO[2])){
    delay(50);
    Serial.print("Orange");
  }
}
``` 

## How it works
After creating and setting up the colour sensor, we need to first set the upper and lower RGB ranges of the goal colour we want to detect. As we know,
we don't have the best accuracy with colours sensors but we can try to isolate the colours we want through trial and error and make sure other colours 
are not detected instead of our goal colour. Here is the definition of the ranges for orange and green detection:
```
  uint16_t clear, red, green, blue;
  uint16_t upperrangeG[3] = {100, 153, 71};
  uint16_t lowerrangeG[3] = {50, 104, 52};
  uint16_t upperrangeO[3] = {216,75,44};
  uint16_t lowerrangeO[3] = {155, 60, 32};  
```

Then we need to calculate the RGB colours between 0 and 255:
```
  // Figure out some basic hex code for visualization
  uint32_t sum = clear;
  float r, g, b;
  r = red; 
  r /= sum;
  g = green; 
  g /= sum;
  b = blue; 
  b /= sum;
  r *= 256; g *= 256; b *= 256;
```

This program prints the RGB values of all colours detected by the sensor but will print "Green" or "Orange" if the colour detected is found within
the specified range limits.

```
 if((r < upperrangeG[0] && g < upperrangeG[1] && b < upperrangeG[2]) || (r > lowerrangeG[0] && g > lowerrangeG[1] && b > lowerrangeG[2])){
    delay(50);
    Serial.print("Green");
  }
```


# References
- Commercial page: https://www.adafruit.com/product/1356
- Online tutorial: https://learn.adafruit.com/adafruit-all-about-arduino-libraries-install-use
- Library: https://github.com/adafruit/Adafruit_TCS34725
- Datasheet: https://cdn-shop.adafruit.com/datasheets/TCS34725.pdf
