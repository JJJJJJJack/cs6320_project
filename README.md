# cs6320_project
This is the package for private cs6320 computer vision project.
The package will work together with ROS and should be used on a Linux system.

## ROS ##
The ROS workspace should be set at location at ```/home/<USERNAME>/catkin_ws```.
For the installation of ROS, please refer to [ROS installation guide](http://wiki.ros.org/kinetic/Installation/Ubuntu)

## OpenCV ##
The C++ node needs OpenCV library and its header. Follow the step below to install OpenCV on your computer. It is assumed that the ROS installation is finished.
> **Dependencies:**

> - ```sudo apt-get install build-essential```
> - ```sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev```
> - ```sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev```

> **Install:**

> - ```mkdir ~/Software```
> - ```cd ~/Software```
> - ```git clone https://github.com/opencv/opencv```
> - ```cd opencv```
> - ```git clone https://github.com/opencv/opencv_contrib```
> - ```mkdir BUILD```
> - ```cd BUILD```
> - ```cmake -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib/modules -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local ..```
> - ```make -j5```
> - ```sudo make install```

After the above step is done, the node should be good to go.

## Running ##
Right now two nodes are reserved. *cs6320_project* will load the image stored in figure folder. *camera* will read the default camera (if there is one) and stream out the edge detection result.
