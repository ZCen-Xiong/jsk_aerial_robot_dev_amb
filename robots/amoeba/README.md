# AMOEBA-Beetle-Art-Omni

## Installation

### 1. Install `acados`

The version of `acados` should be aligned with the version of `3rdparty/acados/CMakeLists.txt` -> `GIT_TAG`.

Specifically, clone the branch for **v0.5.0** from the `acados` git:
```bash
git clone https://github.com/acados/acados.git --branch v0.5.0
```
Then, follow the instructions below:
- Install acados itself: Please follow the instructions on the acados website https://docs.acados.org/installation/index.html
- Install Python interface: Please follow the instructions on the acados website https://docs.acados.org/python_interface/index.html, but don't create virtual env in step 2. The virtual env has compatibility problem with ROS env.
- **Pay attention** that you must execute the step 5 in https://docs.acados.org/python_interface/index.html to test the installation. This step should automatically install t_renderer. If something wrong, please follow step 6 to manually install t_renderer.

### 2. Install the code base and the necessary ROS related packages ...

Setup the folder architecture and clone the repo **with the specific branch**:

```bash
mkdir -p ~/[path_to_ws]/src
cd ~/[path_to_ws]/src
# git clone https://github.com/Li-Jinjie/jsk_aerial_robot_dev.git -b develop/MPC_tilt_mt 
git clone https://github.com/ZCen-Xiong/jsk_aerial_robot_dev_amb.git -b develop/amoeba 
   # pay attention to the branch flag
```

### 2.1 ... for Ubuntu 20.04 and ROS Noetic
Install ROS Noetic for Ubuntu 20.04 from https://wiki.ros.org/ROS/Installation and source the setup file:

```bash
source /opt/ros/noetic/setup.bash
```

We use rosdep to manage the dependencies. So, if you have never done this in your computer before, do the following:

```bash
sudo rosdep init
rosdep update
```

Then, do the following:
```bash
cd ~/[path_to_ws]
wstool init src
wstool merge -t src src/jsk_aerial_robot_dev_amb/aerial_robot_noetic.rosinstall
wstool update -t src    # install unofficial packages
rosdep install -y -r --from-paths src --ignore-src --rosdistro noetic   # install the dependencies/packages stated in package.xml
```

### 2.2 ... for Ubuntu 22.04 and ROS-O
Install ROS-O for ubuntu 22.04 from https://ros.packages.techfak.net/ and source the setup file:

```bash
./jsk_aerial_robot_dev_amb/configure.sh   # for configuration especially for ROS-O in jammy
source /opt/ros/one/setup.bash
```

We use rosdep to manage the dependencies. So, if you have never done this in your computer before, do the following:

```bash
sudo rosdep init
rosdep update
```

Then, do the following:

```bash
cd ~/[path_to_ws]
wstool init src
wstool merge -t src src/jsk_aerial_robot/aerial_robot_${ROS_DISTRO}.rosinstall
wstool update -t src    # install unofficial packages
rosdep install -y -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO      # install the dependencies/packages stated in package.xml
```

For convenience, open `~/.bashrc` and add sourcing of the workspace to the end of the file:

```bash
In ~/.bashrc:

source ~/[path_to_ws]/devel/setup.bash
```

### 3. Install python packages and link them to acados
Install required packages:
```bash
pip install -r src/jsk_aerial_robot_dev_amb/aerial_robot_control/scripts/requirements.txt
```

For the first run, **uncomment** these code in `aerial_robot_control/scripts/nmpc/gen_nmpc_code_all.sh`
```bash
MODELS=(
    NMPCFixQdAngvelOut
    NMPCFixQdThrustOut
    NMPCTiltQdNoServo
    NMPCTiltQdServo
    NMPCTiltQdServoDist
    NMPCTiltQdServoImpedance
    NMPCTiltQdServoThrustDist
    NMPCTiltQdServoThrustImpedance
    NMPCTiltTriServo
    NMPCTiltBiServo
    NMPCTiltBi2OrdServo
    MHEWrenchEstAccMom
)
```

### 4. Build the workspace with `catkin`

```bash
cd ~/path_to_ws
catkin build
```

A frequent problem is the handling of the jobservers in the build process. When occuring during the build process - especially in the aerial_robot_control package - please try simply running the build command again.

### 5. If the build is successful, comment the code in step 3 back.

## Simulation

### 1. Start the simulation
Run the simulation with the following command:
```bash
roslaunch amoeba bringup_nmpc_omni.launch real_machine:=false simulation:=True headless:=False nmpc_mode:=0 batter:=1
```
### 2. Start the keyboard script
Run the keyboard with the following command:
```bash
rosrun aerial_robot_base keyboard_command.py
```
Then input `r` to arm the motors and input `t` to let the drone take off.

### 3. Transformation
**After the main window printed 'Hovering'**:
```bash
rosrun aerial_robot_planning amb_trans.py len=75
```
$len=50$ is equal config.(cross config), $len \in [0,100]$


### 4. Send the trajectory
**After the main window printed 'Hovering'**, please run the following command to send a trajectory:
```bash
rosrun aerial_robot_planning mpc_smach_node.py beetle1
```
Then choose the trajectory you want the drone to perform.

To turn a valve near a wall, you need send trajectory and transformation synchornously
```bash
rosrun aerial_robot_planning rotation_mpc.py beetle1 loop_num=1
```


## Experiments

Most commands are the same with simulation.

For using wrench sensor, call the following command in hovering to calibrate the wrench sensor:
```bash
rosservice call /cfs_sensor_calib "{}"
```
in case your launch don't have the sensor launch, it's:
```bash
roslaunch cfs_sensor cfs_sensor.launch type:=PFS055YA501U6 port:=/dev/ttyACM0
```
if you don't know the port

```bash
lsusb
# or 
ls -l /dev/serial/by-id/ 
```
# Flying Hand

We need two notebooks, one as ground station and the other for visual feedback.

**MoCap Computer**

1. Run the software for the data glove and calibrate it. Note that the ip should be set to the ros master computer.
2. Check the MoCap is set correctly.

If there is something wrong with glove connection, please refer
to https://stretchsense.my.site.com/defaulthelpcenter26Sep/s/article/Studio-Glove-and-Dongle-Setup?language=en_US

**Onboard Computer**

Four terminals are needed.

Before takeoff, launch necessary files

1. `roslaunch beetle_omni bringup_nmpc_omni.launch nmpc_mode:=0 wrench_est_mode:=none  # This is the main launch file for the robot.`
2. `roslaunch aerial_robot_planning hand_arm_glove.launch`
3. `rosbag record -a`

After the robot reach the status of hovering - see terminal output of main launch file -,

4. Run `rosrun aerial_robot_planning mpc_smach_node.py beetle1`, and press 'h' to enter the hand control mode. Note that
   we need to wear the hand part, the shoulder part, and gloves to control the robot.
5. To modify the `/hand/control_mode` parameter using `rosparam` in the terminal, run the following command:
    `rosparam set /hand/control_mode <mode_value>`
    Where `<mode_value>` can be set as one of the following:
    - **1**: Operation Mode
    - **2**: Spherical Coordinate Mode
    - **3**: Cartesian Coordinate Mode
    - **4**: Lock Mode
    - **5**: Exit Mode

    To check the current mode, you can run:`rosparam get /operation_mode`
    
**Ground Station**
Three terminals are needed.

Before takeoff:

1. run `sudo ds4drv` to connect the joystick.
2. `roslaunch aerial_robot_base joy_stick.launch robot_name:=beetle1`
3. `rviz -d ~/ros1/jsk_ws/src/jsk_aerial_robot_dev/robots/beetle/config/nmpc.rviz`

**Transform command**

1. `rosrun aerial_robot_planning amb_trans.py len=50` 
2. len is ranging from 0 to 100, corresponding to negative max and positive max.

***Transformed Valve turning***
1. `rosrun aerial_robot_planning rotation_mpc.py beetle1 loop_num=1` 
2. default is a wall parallel to the y axis of the robot self frame
   
**Overactuated command flight**
1. then `roslaunch aerial_robot_base joy_stick.launch robot_name:=beetle1` or `rosrun aerial_robot_base keyboard_command.py` take off 
2. in 2nd terminal set flight  pos`rosrun aerial_robot_planning agg_state.py robot_name=beetle1` 
3. in 2nd terminal then goes ` roll=90 pitch=0 yaw=nan` (roll=90d eg)
   
    *in this mode, yaw of the command is disabled.*
4. when send `l` landing, will automatically go  to 0 roll and 0 pitch 
