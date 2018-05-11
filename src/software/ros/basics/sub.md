# Subscriber Node
In this chapter we'll create our first listener node, which will retrieve info from another publishing node.

Make sure your current path is the previously created package.

### Create a python file
```
touch listener.py
```

### Make it executable
```
sudo chmod +x listener.py
```

### Add the following code
```python
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('chatter', String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
```

First we initialize the `listener` node and then tell it to subscribe to the previously created `chatter` node.
Here we also need to mention what to do with any incoming data from the `chatter` node. In this case, this function is called `callback`.

Now anytime we get incoming data from the subscribed `chatter` node, the `callback` function will log the data.

Note here that we specify the type of the incoming data (String).

### Test if everything is working
```roscore
rosrun my_awesome_code talker
rosrun my_awesome_code listener

```

You should now be able to see the following output:
```
[INFO] [WallTime: 1439848277.141546] /listener_14364_1439848276913 \ I heard hello world 1439848277.14
[INFO] [WallTime: 1439848277.241519] /listener_14364_1439848276913 \ I heard hello world 1439848277.24
[INFO] [WallTime: 1439848277.341668] /listener_14364_1439848276913 \ I heard hello world 1439848277.34
[INFO] [WallTime: 1439848277.441579] /listener_14364_1439848276913 \ I heard hello world 1439848277.44
```
