# rpi/ultrasound.py

In this package, all the data sent by the ultrasonic sensors are read and if the value of one sensor in front on the robot drops below a certain value (10cm in our case), it tells the arduino to pause its current action. Only when all the sensors' readings are acceptable will the robot be able to move. This is necessary for the qualifications of the robot before it can even go on the battlefield.

The way all this is done is pretty simple and works well. Let's analyse this program from the bottom up.

The following lines just tells our program to execute the `listener()` function  defined above.
```python
__name__ == '__main__':
    listener()
```

Because we have 7 ultrasonic sensors each publishing on their own topic, we must subscribe to those 7 topics individually. In addition, we have to initialise the rospy node and spin it up. This is done respectively with the first and last line of the `listener()` function.

We also subscribe to the `run` topic, which we use to make sure the robot doesn't move after the match time is over.
```python
def listener():
    rospy.init_node('ultrasound', anonymous=True)
    rospy.Subscriber('run', String, stop_cb)
    rospy.Subscriber('ultrasound_1', Range, callback, 1)
    rospy.Subscriber('ultrasound_2', Range, callback, 2)
    rospy.Subscriber('ultrasound_3', Range, callback, 3)
    rospy.Subscriber('ultrasound_4', Range, callback, 4)
    rospy.Subscriber('ultrasound_5', Range, callback, 5)
    rospy.Subscriber('ultrasound_6', Range, callback, 6)
    rospy.Subscriber('ultrasound_7', Range, callback, 7)
    rospy.spin()
```

When a message appears on one of the sensor's topic, it calls the same `callback()` function with one extra argument. That argument is there so we know which sensor is seeing the data that is being passed. This way we cam make sure that none of the sensors has an obstacle in front of it before we tell the robot it can move.

The way it works is that we declared a list `go_condition` that contains all boolean values. Each value corresponds to one sensor and is set to `True` when the sensor is clear and to `False` if an obstacle is present. Only when each element of that list is `True` will it continuously send the "start" string to the `cmd_stop` topic. If that condition is not met, it will send a "stop" message on the same topic. On the Arduino's side, that topic is being listened to and a boolean variable is set which will prevent going into the next iteration of the PID regulation loop and set the motors speed to 0 instead (while staying in that control loop with all other variables unaffected). The effect of this is that when the obstacle is removed and the "start" string is the one bieng published again, the robot simply continues where it had left as if nothing happened.
```python
def callback(data, sensor_num):
    pub = rospy.Publisher('cmd_stop', String, queue_size=10)

    if (data.range < 10 and data.range > data.min_range):
        go_condition[sensor_num] = False
    else:
        go_condition[sensor_num] = True

    if (all(go_condition) == True):
        pub.publish("start")
    else:
        pub.publish("stop")
```

This other callback function is used to set the first entry in the `go_condition` list to false if the "stop" message has been received when the time is over.

The effect of this is the same as if one of the ultrasonic sensors had an obstacle in front of it idefinitely, keeping the robot still until everything is reset.
```python
def stop_cb(msg):
    print(msg.data)
    if (msg.data == "stop"):
    go_condition[0] = False
```
