# Install docker
To ease the development process, we recommend installing ROS inside a docker container instead of using a virtual machine. This comes with several advantages like ease of installation, having a common development environment and performance gains while compiling and running code on your computer.

It is important to note that the robot will not use docker and will run ROS packages natively. If you are an expert and know what you are doing, feel free to use your own method to develop ROS packages. Be it inside a virtual machine or a dedicated linux partition on your drive. Otherwise, just stick to the instructions below.

## Linux
The installation varies quite a bit depending on the distribution you are using because the repositories are different for each distribution.
For more information, visit [this link](https://docs.docker.com/install/#server) and choose your distribution. If your distribution is not listed, try looking at the documentation of your own distribution.

## Windows
On windows, you can download and install [this executable](https://download.docker.com/win/stable/Docker%20for%20Windows%20Installer.exe).
Once the docker app is installed, run it and you should see the docker icon in your system tray. Always make sure the service is up and running before trying to do anything with it.

For more information, follow [this link](https://docs.docker.com/docker-for-windows/install/#install-docker-for-windows-desktop-app).
## macOS
Download and run [this image](https://download.docker.com/mac/stable/Docker.dmg). When asked, drag and drop the docker.app whale icon to the application folder. Once it is finished, simply start the docker service by running the docker app. You should see the docker icon in your system tray. Always make sure the service is up and running before trying to do anything with it.

For more information, follow [this link](https://docs.docker.com/docker-for-mac/install/).

# Download the ROS container
Once docker is properly installed and the service is running, we can download the ROS image.
To begin, open a terminal (or docker cmd on windows) and enter the following command: `docker pull ros`
This will start the download of the official ROS image from the docker hub.

# Run the container
When all of the files have been downloaded, run the container with the following command:
`docker run -it --name=robot ros`. 

This command does several things. First of all, `docker run` creates a new container based on the ROS image we just downloaded and starts it. The `-it` parameter spins up an interactive bash shell inside the container and attaches it to the current terminal. Finally, the `--name=robot` renames the container to something easy we can work with later. You can give this any name you'd like.

You now have a functionning ROS environment.
To exit the container, use <kbd>ctrl</kbd> + <kbd>d</kbd> or type `exit` in the terminal.

Now, if we want to get back into the container we were just using, we can't use the same command as above as it will again create a new container just like before and all our modifications would be lost.
Instead we need to start the previous container again and attach to it. This can be done with the following command:
`docker start robot && docker attach robot`
The `&&` tells our shell to execute the second command once the first one completed successfully.

You should now be back into the bash shell inside the container.
To start working on ROS projects, we recommend creating an appropriate workspace inside the /home/ros directory like so:
`mkdir -p /home/ros && cd /home/ros`

You can now follow the instructions to [setup your workspace](software/ros/basics/workspace.html).

# Additional notes
## Run multiple shells inside the container
By default, only one bash session is running inside the container. Practically, this means that if you try to attach the same container in multiple terminals simultaneously, you won't be able to run separate commands in each.

To fix this -- and this will be needed to launch `roscore` in background before any other ROS command --  start a new bash session inside the container by typing the following in a new terminal:
`docker exec -it robot bash`

This will start a new bash session inside the already running container and attach to it automatically. If you exit this session, the container will keep running in the background. To stop the container, you will have to exit from the terminal where you started (or attached) to it. You can also stop a container with the command `docker stop robot`.
