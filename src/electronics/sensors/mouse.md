# The Mouse

This is a pc mouse.
It gives a travel distance in x-axis and y-axis. It can't see rotation.

# Application

If the mouse is connect in a Linux os, we can read the values in `dev/input/mice`.

The distance given is not a distance in cm, we need to use a multiplication parameter
to have the distance in cm.

## Pros

- Distance traveled compute on the field.

## Cons

- The raspberry must be fast enough.
- It's not better than the encoder of the mechanum wheels.
- It didn't see the rotations.
