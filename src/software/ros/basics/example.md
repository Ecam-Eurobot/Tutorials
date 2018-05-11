# Raspberry led blinking example

Let's now create a small project for the Raspberry Pi, that uses ROS to turn on or off a led connected to it's GPIO's.

For this, we'll need a publisher node that acts like the brain of our system. The calculation and decisions are done on this node. The subscriber node will intercept the message and depending it's value, it will turn the led on or off.

Create a python program and name it **controller.py**.

We'll create a node called **controller** and send the led value to the **led_value** topic. This value will be incremented every second.

```python
import rospy
from std_msgs.msg import Int

count = 0

def talker():
    pub = rospy.Publisher('led_value', Int, queue_size=10)
    rospy.init_node('controller', anonymous=True)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        count += 1
        pub.publish(count)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
```


Create a python program and name it **led1.py**.
We'll create a node called **led1** and intercept the **led_value** topic. If it's value is even, it will turn the led on. If it's odd, the led is turned off.

```python
import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Int

count = 0

def callback(data):
    if data.data % 2:
      GPIO.output(18,GPIO.HIGH)
    else:
      GPIO.output(18,GPIO.LOW)

def listener():
    rospy.init_node('led1', anonymous=True)
    rospy.Subscriber('led_value', Int, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(18,GPIO.OUT)
        listener()
    except rospy.ROSInterruptException:
        pass
```
