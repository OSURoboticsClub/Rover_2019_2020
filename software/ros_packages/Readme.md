# Rover Software
This code is what runs on the NUC onboard the Rover.
This handles everything from processing vision data to actually sending drive commands to the wheels.



## Requirements
**IMPORTANT:**  Read the "How to Launch" section below before trying to configure ROS Kinetic and MoveIt! by yourself. Ignoring this advice will likely result in the creation of multiple catkin workspaces.
* Ubuntu 16.04
* Python 3.X
* [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
* [MoveIt!](http://docs.ros.org/kinetic/api/moveit_tutorials/html/index.html)



## How to Launch | Rover Arm using RViz and MoveIt!
**ALSO IMPORTANT:**  The following instructions assume you don't have ROS Kinetic or MoveIt! installed yet. If you have pre-existing catkin workspaces (e.g., "\~/catkin_ws", "\~/ws_moveit", etc.) back them up by moving them to another location. For example:
```
mv ~/catkin_workspace ~/_catkin_workspace
```
If you've installed ROS/MoveIt! before, you will get a lot of messages indicating that no changes were made. That's fine - just run them all anyway.

<hr>

### 1. ROS Kinetic | Installation
It is recommended you use Xenial (Ubuntu 16.04) as your Operating System (OS) for this installation guide. 
(Though Wily (Ubuntu 15.10) and Jessie (Debian 8) are also supported by ROS Kinetic.)

#### 1.1 Configure your Ubuntu repositories
Configure your ubuntu repositories to allow "restricted," "universe," and "multiverse." ([Click here](https://help.ubuntu.com/community/Repositories/Ubuntu) for more details.)

#### 1.2 Setup your sources.list
Setup your computer to accept software from packages.ros.org.
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

#### 1.3 Setup your keys
```
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
```
If you experience issues connecting to the keyserver, you can try substituting hkp://pgp.mit.edu:80 or hkp://keyserver.ubuntu.com:80 in the previous command.

#### 1.4 Installation
First, make sure your Debian package index is up-to-date:
```
sudo apt-get update
```

**Desktop-Full Install: (Recommended)**: ROS, rqt, rviz, robot-generic libraries, 2D/3D simulators, navigation and 2D/3D perception
```
sudo apt-get install ros-kinetic-desktop-full
```

To find available packages, use:
```
apt-cache search ros-kinetic
```

#### 1.5 Initialize rosdep
Before you can use ROS, you will need to initialize rosdep. rosdep enables you to easily install system dependencies for source you want to compile and is required to run some core components in ROS.
```
sudo rosdep init
rosdep update
```

#### 1.6 Environment Setup
It's convenient if the ROS environment variables are automatically added to your bash session every time a new shell is launched:
```
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### 1.7 Dependencies for building packages
Up to now you have installed what you need to run the core ROS packages. To create and manage your own ROS workspaces, there are various tools and requirements that are distributed separately. For example, rosinstall is a frequently used command-line tool that enables you to easily download many source trees for ROS packages with one command.

To install this tool and other dependencies for building ROS packages, run:
```
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

*Optional:*
Install ROS Tutorials.
```
sudo apt-get install ros-kinetic-ros-tutorials
```

***Congratulations - you are finished with the installation of ROS Kinetic!***

<hr>

### 2. MoveIt! | Installation

#### 2.0 Verify ROS Kinetic was setup properly
Run the following command. If nothing is returned, refer to section 1.6.
```
printenv | grep ROS
```

It is easy to miss steps when going through the ROS installation tutorial. If you run into errors in the next few steps, a good place to start is to go back and make sure you have installed ROS correctly. Once you have ROS installed, make sure you have the most up to date packages:
```
rosdep update
sudo apt-get update
sudo apt-get dist-upgrade
```

Install catkin (the ROS build system):
```
sudo apt-get install ros-kinetic-catkin python-catkin-tools
```

#### 2.1 Install MoveIt!
The simplest way to install MoveIt! is from pre-built binaries (Debian):
```
sudo apt install ros-kinetic-moveit
```

#### 2.2 Create a catkin workspace
You will need to have a catkin workspace setup. If you don't use the name "catkin_workspace" you will run into issues:
```
mkdir -p ~/catkin_workspace/src
```

#### 2.3 Download the example code
Within your catkin workspace, download these tutorials:
```
cd ~/catkin_workspace/src
git clone -b kinetic-devel https://github.com/ros-planning/moveit_tutorials.git
```

You will also need a ```panda_moveit_config``` package to follow along with these tutorials:
```
git clone -b kinetic-devel https://github.com/ros-planning/panda_moveit_config.git
```

#### 2.4 Clone this repository
If you haven't already, create a folder and clone this repository:
```
mkdir ~/Github
cd ~/Github
git clone https://github.com/OSURoboticsClub/Rover_2019_2020.git
```
#### 2.5 Create links to the ROS Packages
To keep the code in the repository, and allow catkin the ability to access it, we will need to create some symbolic links:
```
cd ~/catkin_workspace/src
ln -s ~/Github/Rover_2019_2020/software/ros_packages/rover_arm_moveit_config rover_arm_moveit_config
ln -s ~/Github/Rover_2019_2020/software/ros_packages/mr1718-arm-urdf_export mr1718-arm-urdf_export
```

#### 2.6 Build your catkin workspace
The following will attempt to install from Debian any package dependencies not already in your workspace:
```
cd ~/catkin_workspace/src
rosdep install -y --from-paths . --ignore-src --rosdistro kinetic
```

The next command will configure your catkin workspace:
```
cd ~/catkin_workspace
catkin config --extend /opt/ros/kinetic
catkin build
```

Source the catkin workspace:
```
source ~/catkin_workspace/devel/setup.bash
```

Add the previous command to your .bashrc:
```
echo 'source ~/catkin_workspace/devel/setup.bash' >> ~/.bashrc
```

***Congratulations - you are now ready to rock and robot!***

<hr>

### 3. LaunchIt!

#### 3.1 Run the demo
To run the demo for the rover arm through RViz using MoveIt!:
```
roslaunch rover_arm_moveit_config demo.launch
```

Feel free to try ```roslaunch``` on the other launch files, which can be found here:
```
ls ~/Github/Rover_2019_2020/software/ros_packages/rover_arm_moveit_config/launch
```

<hr>

## Built With
* [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
* [MoveIt!](http://docs.ros.org/kinetic/api/moveit_tutorials/html/index.html)

## Authors
**Adam Stewart** - *Combined documentation from [install ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) and [install MoveIt!](http://docs.ros.org/kinetic/api/moveit_tutorials/html/index.html)* - [GitHub](https://github.com/AdamTogether)


