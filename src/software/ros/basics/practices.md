# ROS: good practices to code with ROS

In this tutorial, you will learn the good paractices to code with the Robot Operating System (ROS).

We never learn better than an example.


## Example of the start test

1. To include the developement environment (python in this example).

```python
#!/usr/bin/env python
```


2. To import all the librairies.

```python
import RPi.GPIO as GPIO
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Empty
```


3. To set the global variables.

```python
left = 0
start = 0
step = 1
```


4. To do the basic setup.

```python
GPIO.setwarnings(False) # Ignore warning
GPIO.setmode(GPIO.BOARD) # Use physical pin numbering

pins = [38, 37] # The used pin(s)
#button0 - side - 38
#button1 - start - 37

for pin in pins:
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) # Set pins to be an input pin and set initial value to be pulled low (off)
```


5. To define the ROS publishers.

```python
bob_pub = rospy.Publisher('show_bob', Empty, queue_size=10)
nav_pub = rospy.Publisher('required_coords', Point, queue_size=10)
```


6. To definition the functions.

```python
def show_bob():
    global bob_pub
    bob_pub.publish(Empty())



def is_left():
    global left
    global step
    if GPIO.input(pins[0]) == GPIO.LOW:
        left = 1
    else:
        left = 0
    if step == 1:
        show_bob()



def is_started():
    global start
    if GPIO.input(pins[1]) == GPIO.LOW:
        start = 1
    else:
        start = 0



def go_to(x, y):
    global nav_pub
    nav_pub.publish(Point(x, y, 0))



def step_ok_callback(data):
    if data.data == True:
        global step
        global left
        if step == 1:
            if left == 1:
                go_to(0, -30) # x and y: the coordinates of the left dispenser (1)
            else:
                go_to(0, 30) # x and y: the coordinates of the right dispenser (1)

        if step == 2:
            rospy.loginfo("Finished !")

        if step > 2:
            rospy.loginfo("Anormally finished !")
        step += 1

    else:
        rospy.loginfo("Waiting ...")
```


7. To write the main code.

```python
if __name__ == '__main__':
    try:
        rospy.init_node("raspberry", anonymous=True)
        rospy.Subscriber('curry_arrived', Bool, step_ok_callback)
        rate=rospy.Rate(1)

        while not rospy.is_shutdown():
            is_started()
            if start == 1:
                is_left()
            rospy.loginfo(step)
            rate.sleep()

            else:
                print('Not started')

    except rospy.ROSInterruptException:
        pass
```


## Explainations of the code

We always check if the start cable is off (if the robot can start).

When it is OK, we check the position of the side button (left or right of the game area) and publish a toggle message on *show_bob* topic to set the suction cup.

After this move, the first arduino send "True" to the *curry_arrived* topic and the callback function is launched. An instruction will be sent after each True message recieved on the same topic.
