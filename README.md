# Dual-Arms
## Dual arms in robotic club Tishreen university
## _by Amin Fayez Haydar_
### install:
#### 1) Clone & Build the project
```bash
git clone https://github.com/Amuntu/Dual-Arms.git
cd Dual-Arms-Kinematics
catkin_make
```
#### 2) Installing _rosserial_arduino_ package
This package is responsible to handle all the communication between Arduino and ROS.
The advantage of using this package over any other serial library is that it allows the user to make Arduino a node in ROS network.
This gives Arduino the capability to publish and subscribe to topics which are there in the network without any hassle.
As mentioned on the official [page](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup), there are two different ways to install this package:
 1. Install from binaries. Run the following commands in the terminal in order to install the package.
      * `sudo apt-get install ros-<distro>-rosserial-arduino`
      * `sudo apt-get install ros-<distro>-rosserial`
  2. In case the method mentioned above doesn't work, the package can be built from source as follows.
      * `cd Dual-Arms-Kinematics/src`
      * `git clone https://github.com/ros-drivers/rosserial.git`
      * `cd ..`
      * `catkin_make`
