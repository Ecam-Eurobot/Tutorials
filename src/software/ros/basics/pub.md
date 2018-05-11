# Publisher Node
As shortly mentioned in the basic concept part, ROS allows interaction pulisher nodes and subscriber nodes.
In this chapter we'll create our first publisher node, which will send a basic message.

Make sure your current path is the previously created package.

### Create a python file
```
touch talker.py
```

### Make it executable
```
sudo chmod +x talker.py
```

### Add the following code
```python
import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
```

First we create the publisher node with reference 'chatter' and initialize it.

Then we go through a loop and create a 'hello world' string that we will publish on this 'talker' node.

`rospy.loginfo()` is used to debug info to the user but doesn't interact with the node itself.

We could now run `roscore` and `rosrun my_awesome_code talker` in the console and see the debugger info but we don't have anybody subscribed(listening) to that `talker` node yet.
