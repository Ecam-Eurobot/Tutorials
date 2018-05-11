# Color detection

>The objective of this module is to detect a color combination and send it to the “Minus” robot module whose function is to build a tower made of cubes.

## Getting started

### Image treatment module

#### Summary 

First and foremost, we must connect to GoPro wifi to take a picture of the color combination. Here, we have chosen a GoPro as a camera because it has an acceptable quality (780 pixel-HD) and it has also a Wi-Fi module for the remote control.

Once the photo has been taken, the camera automatically switches off and we connect to the robot by wifi (a Raspberry connected with the same Wi-Fi for future image treatment).

During this treatment, we use the OpenCV free graphics library.

We have drawn three frames on the picture (one for each cube to detect) to delimit the test portion. The board analyzes all pixels within each frame to obtain the color. 

However, we have implemented in our program the different possible combinations to save time. Indeed, we only required two colors to deduce the final combination.

As a matter of conclusion, we send combination to the robot by ROS. 

![z](img/software/image-processing/image1.png)
#### Bill of materials: 

- One Raspberry Pi zero W
- One Wi-Fi GoPro (Hero+) 
- One 5V battery
- Etcher (https://etcher.io/) to install an OS on the Raspberry

#### Connections

- No pins required  

#### Setup 
- Download last version Ubuntu mate (16.04) or an another distribution for the raspberry (see [install](ros/install.md))
- Active WiFi and open terminal (***alt+ctrl+t***):
	1) Install ROS (see [install](ros/install.md))
	2) Download this repo from [github](https://github.com/Ecam-Eurobot/Tutorials/archive/master.zip), unzip Tutorials-master.zip to open the main folder. 
	3) Drag the folder in the terminal, you'll see a url. Then run this command :
		```
		cd "url"
		```
	
 	4) Run this code above to go to "codes" folder: 
		```
		cd src/codes/software/image-processing/tests
		```
	5) Connect the GoPro (WiFi) or your webcam (USB port) to the Raspberry 

### Checking camera
Check your camera is working. Type the following command line into your prompt and press enter : 
	
1) Go to "test camera" folder : 

	
	cd src/codes/software/image-processing/tests/test_camera
		

**Checking GoPro**

We use a library called "goprocam" that contains all functions to control GoPro 

- Run this command to test GoPro in your terminal  

	```
	python3 go_pro_test.py
	```
	***Make sure your Raspberry is on the same WiFi network as GoPro !***

The code : 

	
	
	#go_pro_test.py

	from goprocam import GoProCamera
	from goprocam import constants

	# Connect to GoPro
	gpCam = GoProCamera.GoPro()

	# Take a photo and save it in the current folder 
	gpCam.downloadLastMedia(gpCam.take_photo(0))
	
	
**Checking webcam**

If you don't have a GoPro, you can also use a webcam. Type the following command line into your prompt and press enter : 

We use here "OpenCV" library to display and save images from the webcam

- Run this command to check the webcam
 
	```
	python webcam_test.py
	```	

The code : 

	
	# webcam_test.py

	import cv2 

	# Create an object called camera and connect the first camera to the computer
	camera = cv2.VideoCapture(0)

	# Take a photo and save it in the current folder 
	return_value, image = camera.read()
	cv2.imwrite('opencv'+'.png', image)


**Result** : 
	![z](img/software/image-processing/image5.png)


## Scripts ## 

After checking the camera, we must run a script to automate the color detection and the WiFi switching (if you use a GoPro).  
This script makes three actions :  
1) To switch WiFi to connect to GoPro to obtain a picture
2) To send the picture to the Raspberry to start the image processing
3) To give a color combination 

Type the following command line into your prompt and press enter to check the settings and run the script 

- Go to color_detection folder :
```
cd ../test_color_detection
```
- Open GP_combination.sh :
```
nano GP_combination.sh 
```  
- Change the name with your GoPro WiFi :
```
nmcli c up "your GoPro wifi"
```
- Change the name with your robot WiFi 
```
nmcli c up "your wifi robot"  
```
save it (***ctrl+x***)

- Run this script on the terminal : 
```
bash GP_combination.sh 
``` 
You'll see 3 colors frames on the picture and the final combination color on the terminal)
![z](img/software/image-processing/image6.png)

The code : 
```
#GP_Combination.sh

#!/usr/bin/expect

#switch to "GoPro wifi and connect to the GoPro" 

nmcli c up "armen" 
sleep 10

#Take photo with GoPro
python3 GP_takePhoto.py

#Switch to "robot wifi to connect to minus(robots)

nmcli c up "Airport Express Lenaerts" 

#To obtain the color combination 
python2 color_detection.py
```

If you prefer using the webcam instead of GoPro

```
bash WB_combination.sh 
```
The code : 
```
#WB_combination.sh 

#!/usr/bin/expect

python2 WB_takePhoto.py

python2 color_detection.py
```
**Result** : 
The result is the same for GoPro and webcam. 

You'll see 3 colors frames on the picture and the final combination color on the terminal as shown in the image below

![z](img/software/image-processing/image6.png)

## Image processing - color_detection.py ##

After taking a picture with the camera or the GoPro, we must process it to generate the color combination : 
The program creates three frames, one for each cube and analyzes all pixels within each frame: 
1.	It looks whether the color detected is in the range color defined for every color (each color range is defined with one high and one low BGR value)
2.	After that, it returns a value: more the value is great, more the color lies within the range. The program adds up the values and the final value thus obtained determines the color within the range of colors determined initially. 

**Adjusting the position of a frame**

We must adjust the position of frames to target the part of the image we want to process :  
Each position is represented by a matrice. The first value (pixel) is linked with red frame, the second value (pixel) with yellow frame and the last value (pixel) with white frame. 

![before](img/software/image-processing/image2.png)

*positionName = [red frame value, yellow frame value, white frame value]*

Here is the different positions for each frame: 
 
- **xmin** is the left side of the frame 
- **xmax** is the right side par of the frame 
- **ymin** is the top side of the frame 
- **ymax** is the bottom side of the frame 

**Procedure** : 

1) Maintain your cursor where you want to place a side of the frame
2) Take note of the position (pixel)
3) Open a new tab (***ctrl+shift+t***) and run this command to change a side of a frame in the following code(color_detection.py) : 
```
nano color_detection.py 
```

4) Restart for another side of the frame then save the file (***ctrl+x***)

***Result*** : 

![after](img/software/image-processing/image3.png)

**Adjusting the range color**

We must adjust the color range (RBG) to define each color. Therefore, the programme will be able to identify differents colors in each frame in function of these RGB values. We represente  each color range by a matrix :

*color_range = [blue value, green value, red value]*

As a reminder, an RGB color value is specified with RGB (red, green, blue).
Each parameter (red, green, and blue) defines the intensity of the color as an integer between 0 and 255.
For example, rgb(0, 0, 255) is rendered as blue, because the blue parameter is set to its highest value (255) and the others are set to 0.

**Procedure** : 

1)  Maintain your cursor in the center of the cube 
2) Take note of the RGB color
3)  Open a new tab (***ctrl+shift+t***) and change the color range in the code (color_detection.py) 

```
nano color_detection.py 
```
4) Restart for another cube then save the file (***ctrl+x***)

***Result*** :
 
![z](img/software/image-processing/image4.png)

**Automation**

We have also implemented in our program the different possible combinations to save time. Indeed, we only required two colors to deduce the final combination.


## Application with ROS ###

You use ***ROS*** to send the color combination to another robot module :

1) Create a package "image-processing" and create a launch files" test.launch" (more details [here](ros/install.md))
2) Drag the code "image-analyze.py"(codes/software/image-processing) to catkin_ws/src/"image-processing"
3) Open terminal and launch ROS : 
````
. ~/eurobot_ws/devel/setup.bash
roslaunch image-processing test.launch 
```` 
- Open a new tab (***ctrl+shift+t***) to display messages published to a topic : 
 
```` 
rostopic echo /color_seq 
````

![z](img/software/image-processing/image7.png)

***Don't forget to change frames positions and color ranges ! (see section below)***
### Flashlight module

#### Aim

We added a LED flashlight above the camera because light is an important factor and can bias our previous calculations. Therefore, the constant level of light thus created enables us to maintain our settings. This flashlight is turned off immediately after the camera has taken the photo, so as not to disrupt other teams or distract the audience. 

#### Bill of materials: 

- One Raspberry Pi zero W
- One LED flashlight 
- One 5V battery

#### Connections

We use pin 4 of the raspberry Pi zero to power the gate pin of IRF520 mosfet in order to control the light. The board is powered with a 5V power supply from the battery (pin 2 = 5V and pin 6= GND).

#### Setup 
- for testing the flashlight, run :  
```
python flashlight.py
```
You'll blink the flashlight 
