# Sonar #

A sonar is a sensor allowing you to measure the distance to an obstacle thanks to high frequency waves. To keep correct measured values it is better to use the sensor we chose between 2cm and 400cm. The model of our sonar is HC-SR04. There are 4 pins on this sensor : TRIG, ECHO, VCC and GND (as you can see on the picture below).

![img](img/electronics/sonar/HCSR04.png)

It contains a transmitter and a receiver.
The distance measure is pretty simple. Firstly, you have to send a signal on the TRIG pin so the sonar emits a wave of 8 pulses at 40kHz. Then the ECHO pin is set to the «HIGH» level until the emitted signal goes to the obstacle and comes back to the sensor (as you can see on the following picture).

![img](img/electronics/sonar/ultrasonic_waves.png)

Of course, you have to connect the sensor to a microcontroller. You can connect the VCC to the 5V pin of your board, the GND to the ground pin and the TRIG and ECHO to digital pins of your controller as illustrated on the picture below.

![img](img/electronics/sonar/sonar_pins.png)

Finally it is also important to know that the measuring angle with the HC-SR04 sonar is 30 degrees.  Which is short, so the sonar can only see in front of him

All these information are also explained in the following websites :

<https://www.gotronic.fr/pj2-hc-sr04-utilisation-avec-picaxe-1343.pdf>

<https://wiki.mchobby.be/index.php?title=HC-SR04>

<http://web.eece.maine.edu/~zhu/book/lab/HC-SR04%20User%20Manual.pdf>  

Here is an online store where you can buy this sensor :  

<https://shop.mchobby.be/proximite/561-senseur-ultrason-hc-sr04-3232100005617.html>

Images references :

<http://geii.iut-troyes.univ-reims.fr/wikigeii/index.php?title=Fichier:Working-of-HC-SR04-Ultrasonic-Sensor.jpg>

<http://www.instructables.com/id/HC-SR04-Ultrasonic-Sensor-With-Raspberry-Pi-2/>

<https://randomnerdtutorials.com/complete-guide-for-ultrasonic-sensor-hc-sr04/>

# Tutorial #

## Simple Arduino code

To be able to test the simple code showed in this part you have to connect the sonar sensor to your Arduino Uno board as showed on the third picture hereinabove. You also have to connect the TRIGG pin of the sonar to the pin 12 on your Arduino board and the ECHO pin of the sonar to the pin 13 on the Arduino board. Of course you can change these values on the following code if needed.  
You are now ready to upload the code :

```cpp

#define TRIGG 12 // Broche TRIGGER
#define ECHO 13    // Broche ECHO
                                               // definition du Timeout
const long TIMEOUT = 25000UL; // 25ms = ~8m à 340m/s

float son= 340.0 / 1000; //vitesse du son dans l'air (mm/µs)

void setup() {

  pinMode(TRIGG, OUTPUT);  //Configuration des broches
  digitalWrite(TRIGG, LOW); // La broche TRIGGER doit être à LOW au repos
  pinMode(ECHO, INPUT);

  Serial.begin(9600); //Démarrage de la liaison série
}

void loop() {

  digitalWrite(TRIGG, HIGH); // Lance une mesure de distance en envoyant
  delayMicroseconds(10);  //une impulsion HIGH de 10µs sur la broche TRIGGER
  digitalWrite(TRIGG, LOW);

  int mesure = pulseIn(ECHO, HIGH, TIMEOUT); // Mesure le temps entre
                                          // l'envoi de l'ultrason et sa réception

  float distance_mm = mesure / 2.0 * son; //calcul de la distance grâce au temps
                                      //on divise par 2 car le son fait un aller-retour

  Serial.print("Distance: "); //Affichage des résultats
  Serial.print(distance_mm);
  Serial.println("mm");

  delay(500); //temps entre chaque mesure (ms)
}

```

To verifiy if everything is working correctly after uploading the code you can click on the Serial monitor icon on the Arduino IDE (as shown on the picture below) to see what distance the sonar returns in milimeters.

![img](img/electronics/sonar/serial_monitor.png)

Link to the code on Ecam Eurobot Github :  

<https://github.com/Ecam-Eurobot/Eurobot-2018/blob/ultrasound/arduino/sonar_simple_test_no_ROS.ino>

Go check in ROS and Arduino chapter for combining Sensor, Arduino and ROS 
