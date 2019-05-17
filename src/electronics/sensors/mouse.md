# The Mouse

This is a pc mouse.
It gives a travel distance in x-axis and y-axis. It can't see rotation.

# Application

If the mouse is connect in a Linux os, we can read the values in `dev/input/mice`.

```python
import struct

with open("/dev/input/mice","rb") as fd:
    while True:
        # unpack data to have x and y
        y,x = struct.unpack("xbb",fd.read(3))

```

The distance given is not a distance in cm, we need to use a multiplication parameter
to have the distance in cm.

We test the it on the 2019 eurobot.
You can find the files with ROS in the little_robot branch of Eurobot-2019, `little_robot/ROS_packages/R&D/mouse/src`.

## Pros

- Distance traveled compute on the field.

## Cons

- The raspberry must be fast enough.
- It's not better than the encoder of the mechanum wheels.
- It didn't see the rotations.
