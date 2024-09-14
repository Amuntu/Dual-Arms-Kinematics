# Dual-Arms
## Dual arms in robotic club Tishreen university
## _by Amin Fayez Haydar_
### install:
#### 1) Clone & Build the Project
```bash
git clone https://github.com/Amuntu/Dual-Arms-Kinematics.git
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
      ```bash
      cd Dual-Arms-Kinematics/src
      git clone https://github.com/ros-drivers/rosserial.git
      cd ..
      catkin_make
      ```
#### 3) Run Project
   ##### A) Run ROS-Master:
   ```bash
   roscore
   ```
   ##### B) Source the Package:
   ```bash
   source devel/setup.bash
   ```
   ##### C) Open Serial port for Arduino connected to ypur PC:
   ```bash
   rosrun rosserial_python serial_node.py _port:=<device> _baud:=<baud rate>
   ```
   ##### D) Run the mission you Desire:
   _you have only 4 Missions:_
   * mission = 0 : that means do the IK.
   * mission = 1 : that means do the FK.
   * mission = 2 : that means do the IJ.\n
   * mission = 3 : that means go to Zero.
      ###### 1. In case of running Inverse-Kinematics:
        ```bash
        rosrun DualArms_manual_Package Control_IK_FK_J 0  Px  Py  Pz
        ```
      ###### 2. In case of running Forward-Kinematics:
        ```bash
        rosrun DualArms_manual_Package Control_IK_FK_J 1  R1q1  R1q2  R1q3  R1q4  R1q5  R2q1  R2q2  R2q3  R2q4
        ```
      ###### 3. In case of running Jacobian-Kinematics:
        ```bash
        rosrun DualArms_manual_Package Control_IK_FK_J 2
        ```
      ###### 4. In case of running Move to Zero-Pose:
        ```bash
        rosrun DualArms_manual_Package Control_IK_FK_J 3
        ```
