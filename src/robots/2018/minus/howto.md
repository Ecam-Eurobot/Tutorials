# How to run the robot


## Pendant les tests

To launch Minus, just run the main Minus launchfile that is in the **robot_2018** package.
So just make the following command

```bash
roslaunch robot_2018 minus.launch team:=value
```

If we want to run rviz to see the simulation of the robot just add a parameter.

```bash
roslaunch robot_2018 minus.launch team:=value viz:=true
```

> By default, rviz does not start and if we do not set the team parameter, the default value is unknown.

