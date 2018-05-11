## Starting Board

>*last updated on May 11, 2018*
> 

Inspired by past version of the starting board

The utility of this PCB is to gather the component serving to the start sequence. Normally this is the only interface with the robot during the 
competition.

First, we used three switches for configuring three main points of our robot : 

* Team configuration
We have to setup the starting side for the next match to the robot. It behaviour is different if the robot play as the green or the orange team. 
Consequently, this switch set this point before the match start. 

* Strategy Configuration
It makes possible to choose between two kinds of implemented strategies before the match. For example, just before the start, we have the
possibility of aggressive or defensive strategy, function of the opponent. 

* Initialisation Switch

This switch launches an initialisation sequence. After the power up of the robot and before its first move, we wanted to have the possibility to check 
all different robot's systems. This switch indicates with a DEL if all systems are operational. This is a kind of automatic check list before the match 
and the first move of the robot. 

All these switches are linked to two DEL. First one to electrically confirm switches' position and the associated setting and the second one used to 
have the response of the robot operating system. With these two LEDs, we are sure that our robot understood all these parameters.

### Useful information
* Correspondence

Strategy  

Pin 1 - UP   = GREEN = 0 (out raspberry) - DOWN  = RED = 1 (out raspberry)

Initialisation

Pin 2 - UP   = GREEN = 0 (out raspberry) - DOWN  = RED = 1 (out raspberry)

Team

Pin 3 - LEFT = GREEN = 1 (out raspberry) - RIGHT = RED = 1 (out raspberry)

Start 

Pin 4 - piece IN     = 0 (out raspberry) - piece OUT   = 1 (out raspberry)

* Switch placement

The emplacement is taken when the splayed side is on the right then : 

1. Strategy switch is in the upper left corner 
2. Initialisation switch is on the right side of the strategy switch 
3. Team switch is in the lower left corner 

Finally, the starter switch is plugged on the board. It consists of a relay, sending a binary information to the ROS operating system. Linked to a 0.5m cable, this system allows to launch the match sequence pulling on it. 

### Getting Started

The following code is launched with ROS (if you haven't read yet this section, it might be tricky) this is a simplified version of the code used in 
2018 robots. The aim of this code is the setting of a parameter in a configuration file in the first place, next the code publish on the start topic 
when the start switch is pulled. This last action indicates to all script listening this topic the beginning of a game. 

```python 
import sys
import random
import rospy
from std_msgs.msg import Empty

raspberry = "-r" in sys.argv

pi = None
# If passed the -r argument, load the rapsberry libs
if raspberry:
    import pigpio
    pi = pigpio.pi()

rospy.init_node('startup_conf')

# Robot
robot = rospy.get_param("/robot")

pub_start = rospy.Publisher('start', Empty, queue_size=1)
pub_reset = rospy.Publisher('reset', Empty, queue_size=1)


# GPIO PIN CONFIGURATIONS
# This pin will be used to configure the team
pin_team = 23

# This pin will be used to drive a led to indicate that the team has been set correctly in ROS
pin_team_feedback = 21

# This pin will be used to drive a led to indicate that the start is understood by ROS
pin_strategy_feedback = 20

# This pin will be used to launch robot
pin_start = 12

# Function to set team parameter in the configuration file 
def update_team(gpio, level, tick):
    if raspberry:
        if level:
            rospy.set_param("/team", "red")
            rospy.set_param("/reset/position", rospy.get_param("/start/{}/red/position".format(robot)))
            pi.write(pin_team_feedback, 1)
        else :
            rospy.set_param("/team", "green")
            rospy.set_param("/reset/position", rospy.get_param("/start/{}/green/position".format(robot)))
            
   # Blink 2 times for acknowledge
            pi.write(pin_team_feedback, 0)
    else:
        choice = random.choice(["green", "red"])
        team = rospy.set_param("/team", choice)
        rospy.logwarn("Setting team automatically to: " + choice)
        if team == "green":
            rospy.set_param("/reset/position", rospy.get_param("/start/{}/green/position".format(robot)))
        elif team == "red":
            rospy.set_param("/reset/position", rospy.get_param("/start/{}/red/position".format(robot)))

    reset()


publish = True

# If high level on start pin this function publish on the start topic
def start(gpio, level, tick):
    global publish
    rospy.sleep(0.2)
    if raspberry and not pi.read(pin_start):
        if publish :
            pi.write(pin_strategy_feedback, 1)
            pub_start.publish(Empty())
            publish = False

# Function to reset all set parameters
def reset():
    pub_reset.publish(Empty())
    if raspberry:
        if pi.read(pin_start):
            global publish
            publish = True
            pi.write(pin_strategy_feedback, 0)

com = False
init = True

if raspberry and init:
    pi.set_mode(pin_team, pigpio.INPUT)
    pi.set_mode(pin_start, pigpio.INPUT)
    pi.set_mode(pin_team_feedback, pigpio.OUTPUT)
    pi.set_mode(pin_strategy_feedback, pigpio.OUTPUT)
    pi.set_pull_up_down(pin_start, pigpio.PUD_DOWN)
    pi.set_pull_up_down(pin_strategy_feedback, pigpio.PUD_DOWN)
    teamInterrupt = pi.callback(pin_team, pigpio.EITHER_EDGE, update_team)
    startInterrupt = pi.callback(pin_start, pigpio.EITHER_EDGE, start)

    update_team(None, pi.read(pin_team), None)
else:
    rospy.sleep(1)
    update_team(None, None, None)

while not rospy.is_shutdown():
    pass
```

Below, you have the circuit to implement  between the board and the raspberry running ROS

![Start PCB Front](electronics/pcb/pictures/Start_F.jpg)

* The two couples of black and red wires are the 5V 
* The white wire is connected to the pin 12 on the raspberry and send startup interrupt information
* The red point is the pin normally connected to the 23rd pin on the raspberry and send team switch information
* The brown and green wires are connected to the pins 21 and 20 on the raspberry and connect acknowledge pins


If you copied the code and connect all cable as the pictures shows, the pulling of the start cable induce a publication on the start ROS topic (visible 
with an echo on this topic) and the modification of the switch place sets the team parameter ROS file. 

The switch strategy and initialisation were planned to respectively set a different implemented strategy and launch a initialisation (tests) 
sequence. It was not used in 2018 due to a lack of time but they are already on the PCB for a future robot.

Finally, in the real implementation on ROS, the pin configuration is placed in a .yaml file gathering all information required about raspberry pinout.
Then, the code used to get pin number is 

```python
rospy.get_param("/path/name")
```

[Github link for code](https://github.com/Ecam-Eurobot/Eurobot-2018/blob/differential_driver/ros_packages/strategy/src/startup_conf.py)

--- [OneDrive link for Altium project](https://)
