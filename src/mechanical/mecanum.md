# Mecanum wheels

![](img/mechanical/mecanum/mecanum.jpg)

A mecanum wheel is a wheel with rollers attached at its circumference at an angle of typically 45°.
When four mecanum wheels are used together, we can achieve a net resulting direction in **any** direction by varying the direction and speed of rotation of the wheels.


## Assembly
### Wheel adapter
To mount the wheels onto the motors, we need to use an adapter.
Due to an error in our order, we didn't receive the correct adapters and the ones we received didn't fit on the wheels.

We decided to 3D print our own as a replacement. The [STL file for this part can be found here]().

> **Note:**  
3D printing parts that will be under moderate or heavy mechanical stress is often not the best idea, because those parts will tend to break and/or wear out quickly. 


### Wheel orientation

When assembling the wheels onto the robot, we need to pay attention to their orientation.
If we look from the top view, the rollers should all point to the center of the base.

![](img/mechanical/mecanum/mecanum-orientation.png)

## Kinematics

**Forward kinematics** refers to the use of the kinematic equations of a robot to compute the position of the end-effector from specified values for the joint parameters. In our case, the forward kinematics allow us to compute the global velocity of the robots base when given the angular velocities of the individual wheels.

The reverse process that computes the joint parameters that achieve a specified position of the end-effector is known as **inverse kinematics**. The inverse kinematic equations allow us, in our case, to compute the individual wheel velocities needed to achieve a given base velocity.

The equations presented in the next sections come from the following research paper: ["Kinematic Model of a Four Mecanum Wheeled Mobile Robot"](https://research.ijcaonline.org/volume113/number3/pxc3901586.pdf)

### Inverse kinematics

The inverse kinematic equations allow us to compute the indiviual wheel velocities when we want to achieve an overall base velocity.

- \\(\omega_{fl}\\), \\(\omega_{fr}\\), \\(\omega_{rl}\\) and \\(\omega_{rr}\\)  represent the *angular velocities* for the front left, front right, rear left and rear right wheel respectively.
- \\(v_x\\) and \\(v_y\\) represent the robot's base linear velocity in the x and y direction respectively. The x direction is in front of the robot.
- \\(\omega_z\\) is angular velocity of the robot's base around the z-axis.
- \\(l_x\\) and \\(l_y\\) represent the distance from the robot's center to the wheels projected on the x and y axis respectively.

<!-- MathJax math equation -->
\[
    \left\{\begin{matrix}
    \omega_{fl} &= \frac{1}{r} \left[v_x - v_y - (l_x + l_y)\omega_z \right ] \\ 
    \omega_{fr} &= \frac{1}{r} \left[v_x + v_y + (l_x + l_y)\omega_z \right ] \\ 
    \omega_{rl} &= \frac{1}{r} \left[v_x + v_y - (l_x + l_y)\omega_z \right ] \\ 
    \omega_{rr} &= \frac{1}{r} \left[v_x - v_y + (l_x + l_y)\omega_z \right ] 
    \end{matrix}\right.
\]

Or in matrix form:

<!-- MathJax math equation -->
\[
    \begin{bmatrix}
    \omega_{fl} \\ 
    \omega_{fr}\\ 
    \omega_{rl}\\ 
    \omega_{rr}
    \end{bmatrix}
    =
    \frac{1}{r}
    \begin{bmatrix}
    1 & -1 & -(l_x + l_y) \\ 
    1 &  1 &  (l_x + l_y) \\ 
    1 &  1 & -(l_x + l_y) \\ 
    1 & -1 &  (l_x + l_y)
    \end{bmatrix}
    \begin{bmatrix}
    v_x\\ 
    v_y\\ 
    \omega_z
    \end{bmatrix}
\]

#### Example

If we want to let the robot move diagonally at `0.22 m/s` in the x direction and `0.11 m/s` in the y direction. At what speed should we set the motors of each wheel?

The robot is 15 cm in width and 20 cm in length. The wheels are placed at the extremities and have a diameter of 60 mm.

---
<!-- MathJax math equation -->
\[
    \left\{\begin{matrix}
    \omega_{fl} &= \frac{1}{0.03} \left[0.22 - 0.11 - (0.2 + 0.15) \cdot 0 \right ] = 3.66 \; rad / s\\ 
    \omega_{fr} &= \frac{1}{0.03} \left[0.22 + 0.11 + (0.2 + 0.15) \cdot 0 \right ] = 11 \; rad / s \\ 
    \omega_{rl} &= \frac{1}{0.03} \left[0.22 + 0.11 - (0.2 + 0.15) \cdot 0 \right ] = 11 \; rad/s \\ 
    \omega_{rr} &= \frac{1}{0.03} \left[0.22 - 0.11 + (0.2 + 0.15) \cdot 0 \right ] =  3.66 \; rad/s
    \end{matrix}\right.
\]

--- 

### Forward kinematics

The forward kinematic equations allow us to compute the robot's base velocity when given the individual wheel velocities. This is usefull to compute the robot's odometry using the motor's embedded quadrature encoders. 

*Odometry is the use of sensor data to estimate the change in position of the robot over time.*

<!-- MathJax math equation -->
\[
    \left\{\begin{matrix}
    v_x & = (\omega_{fl} + \omega_{fr} + \omega_{rl} + \omega_{rr}) \cdot \frac{r}{4}\\ 
    v_y & = (-\omega_{fl} + \omega_{fr} + \omega_{rl} - \omega_{rr}) \cdot \frac{r}{4}\\ 
    \omega_z & = (-\omega_{fl} + \omega_{fr} - \omega_{rl} + \omega_{rr}) \cdot \frac{r}{4(l_x + l_y)}
    \end{matrix}\right.
\]

Or in matrix form:

<!-- MathJax math equation -->
\[
    \begin{bmatrix}
    v_x\\ 
    v_y\\ 
    \omega_z
    \end{bmatrix}
    =
    \frac{r}{4}
    \begin{bmatrix}
    1 & 1 & 1 & 1\\ 
    -1 & 1 & 1 & -1\\ 
    -\frac{1}{(l_x+l_y)} & \frac{1}{(l_x+l_y)} & -\frac{1}{(l_x+l_y)}  & \frac{1}{(l_x+l_y)} 
    \end{bmatrix}
    \begin{bmatrix}
    \omega_{fl}\\ 
    \omega_{fr}\\ 
    \omega_{rl}\\ 
    \omega_{rr}
    \end{bmatrix}
\]
