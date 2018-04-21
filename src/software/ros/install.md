# Installing ROS

ROS is only officially supported on Ubuntu or Debian, with other platforms being experimental or unofficial. We recommend installing [Ubuntu](https://www.ubuntu.com/download/desktop) for desktops or laptops and [Ubuntu Mate](https://ubuntu-mate.org/raspberry-pi/) for the Raspberry Pi.

## Installing ROS on a Raspberry Pi

### Installing Ubuntu Mate
[Download Ubuntu Mate](https://ubuntu-mate.org/download/) Xenial 16.04, choosing the Raspberry Pi architecture.

The image is compressed. On Windows you can use a tool like 7-zip to decompress it. On Linux and MacOS you can use the following command:

```
xz -d filename.xz
```

#### Writing the SD card
Now we need to write the image file onto the microSD card. 

On **Windows** we can use [Win32 Disk Imager](https://sourceforge.net/projects/win32diskimager/). 

![Win32DiskImager](../img/software/ros/win32diskimager01.png)

On **Linux** we use the `dd` command.

First we need to find our SD card using the `lsblk` command. When you have found the name, you can unmount it.


```
# Find the SD card
lsblk

# Unmount the SD card
sudo umount /dev/sd<?><?> 

# Copy the image to the SD card
sudo dd if=ubuntu-mate-16.04.2-desktop-armhf-raspberry-pi.img of=/dev/sd<?> bs=4M
```

The `dd` command will take a long time to complete and does not give any feedback. Be patient...

On **MacOS** we also use the `dd` command, but it is slightly different than for Linux.

```
# Find the SD card
diskutil list

# Unmount the SD card
sudo umount /dev/disk<?>s<?>

# Copy the image to the SD card
sudo dd if=ubuntu-mate-16.04.2-desktop-armhf-raspberry-pi.img of=/dev/disk<?> bs=4m
```

<span style="color:#FF0000;">[...]</span>

### Install dependencies

Install Git.
```
sudo apt-get install git
```

### Configure the Raspberry Pi



### Installing ROS

Once Ubuntu is installed, we need to configure the sources to accept software from the ROS repositories:

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
```


After that, you can install ROS by issuing the following commands:

```
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
```

This may take a while...

>**Tip:** To find available ROS packages you can use the following command:
>```
>apt-cache search ros-kinetic
>```

### Setup ROS

Initialize rosdep:
```
sudo rosdep init
rosdep update
```

ROS needs to setup a lot of environment variables to work properly. It is easier if they are loaded automatically
when you launch a terminal. To do this, we copy the setup code into our `.bashrc` file:
```
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Install dependencies to build packages

```
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential
```


