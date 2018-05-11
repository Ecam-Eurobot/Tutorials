# Mecanum wheels

We introduced the [mecanum wheels](mechanical/mecanum.html) in the mechanical part.
In this chapter we will review a ROS node we made to control the mecanum wheels.

# ROS Twist

In the ROS navigations stack, all movements are indicated by [Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html) messages.
These messages contain linear and angular velocity components and are often used to express the global movement of the base.

To control the 4 motors, we need to convert those velocities in angular velocities for each wheel. In our ROS node, we can define the 
following function:


```python
def convert(move):
    x = move.linear.x
    y = move.linear.y
    rot = move.angular.z

    front_left = (x - y - rot * WHEEL_GEOMETRY) / WHEEL_RADIUS
    front_right = (x + y + rot * WHEEL_GEOMETRY) / WHEEL_RADIUS
    back_left = (x + y - rot * WHEEL_GEOMETRY) / WHEEL_RADIUS
    back_right = (x - y + rot * WHEEL_GEOMETRY) / WHEEL_RADIUS
```

We used the inverse kinematic equations presented in the [mecanum wheels](mechanical/mecanum.html) chapter to convert global base velocity
into individual angular velocities.

## ROS node

Now that we have a function to transform the twist message, let's setup the node to subscribe to Twist messages and publish individual
angular velocity messages to specific topics:

```python
rospy.init_node('mecanum')

# Get parameters about the geometry of the wheels
WHEEL_SEPARATION_WIDTH = rospy.get_param("/wheel/separation/horizontal")
WHEEL_SEPARATION_LENGTH = rospy.get_param("/wheel/separation/vertical")
WHEEL_GEOMETRY = (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH) / 2
WHEEL_RADIUS = rospy.get_param("/wheel/diameter") / 2


pub_mfl = rospy.Publisher('motor/front/left', Float32, queue_size=1)
pub_mfr = rospy.Publisher('motor/front/right', Float32, queue_size=1)
pub_mbl = rospy.Publisher('motor/rear/left', Float32, queue_size=1)
pub_mbr = rospy.Publisher('motor/rear/right', Float32, queue_size=1)

sub = rospy.Subscriber('cmd_vel', Twist, convert)
rospy.spin()
```

In this extract, we initialize the node, get some parameters from ROS' parameter server, define a publisher for each mecanum wheel and 
subscribe to the `cmd_vel` topic where Twist messages are send to. In the subscription, we provide our convertion function as a callback.
This means that whenever a twist message is published on the topic, our function will be called with the twist message as argument.

The only thing left to do is to adapt our function to pusblish the correct values to the corresponding topics.

The code can be found in the Eurobot-2018 repository in the [mecanum package](https://github.com/Ecam-Eurobot/Eurobot-2018/blob/master/ros_packages/mecanum/src/mecanum.py)
