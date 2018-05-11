# Parameter server

Configuration information in ROS is usually saved to the Parameter server. The Parameter sever is a collection of values that can be accessed upon request through the command prompt, nodes or launch files. Parameters are intended to be fairly static, globally available values such as integers, floats, strings or bool values.

## Parameters
Parameters are named using the normal ROS naming convention. This means that ROS parameters have a hierarchy that matches the namespaces used for topics and nodes. This hierarchy is meant to protect parameter names from colliding. 

```
/motors/front/left: 5.0
/motors/front/right: 4.0
/motors/rear/left: 4.0
/motors/rear/right: 5.0
```
The parameter `/motors/front/left` has the value `5.0`. You can also get the value for `/motors/front`, which is the dictionary

```
left: 5.0
right: 4.0
```

And you can also get the value for `/motors`, which has a dictionary of dictionaries representation of the parameter tree:
```
front: { left: 5.0, right: 4.0 }
rear: { left: 4.0, right: 5.0 }
```

## Parameters from the terminal
ROS has a tool called `rosparam` to manage Parameter Server. The accepted parameters are as follows:
* `rosparam set parameter value`: This sets the parameter
* `rosparam get parameter`: This gets the parameter
* `rosparam load file`: This loads parameters from the file
* `rosparam dump file`: This dumps parameters to the file
* `rosparam delete parameter`: This deletes the parameter
* `rosparam list`: This lists the parameter names

For example, we can see the parameters in the server that are used by all nodes:
```bash
rosparam list
```
We obtain the following output for the above example:
```bash
/motors/front/left
/motors/front/right
/motors/rear/left
/motors/rear/right
```
If you want to read a value, you will use the `get` parameter:
```
rosparam get /motors/front/left
```
To set a new value, you will use the `set` parameter:
```
rosparam set /motors/front/left 6.0
```

## Accessing parameters from nodes
It is often the case that your nodes will have to access the parameter server during start up to retrieve configuration information, or set a parameter value. This can be done quite easily in Python, to set a parameter use:

```python
rospy.set_param(/motors/front/left, 6.0)
rospy.get_param(/motors/front/left)
```


## Accessing parameters from launchfiles
The final source where you may need to access the parameter server is from a launch file. Setting a parameter value during a launch file is common practice to conveniently initialize parameters on start up. This can be done in your launch file using

```yaml
<param name="/motors/front/left" value="6.0"/>
```

## Parameters files

We can also create a file containing all the parameters of the project. The file is a yaml file.

```yaml
start:
    cortex:
        green:
            position:
                x: 0.1
                y: -0.50
                orientation: 0
        red:
            position:
                x: 2.90
                y: -0.50
                orientation: 0
    minus:
        green:
            position:
                x: 0.5
                y: -0.85
                orientation: 0.0
        red:
            position:
                x: 2.3
                y: -0.9
                orientation: 3.14159
```
The parameters will be accessible via a dictionary in python.