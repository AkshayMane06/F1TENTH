# F1TENTH

Make Dual Boot Ubuntu 

**Steps to run simulator** 
1. Open Terminal in Ubuntu go to directory and build docker
cd  sim_ws/src/f1tenth_gym_ros
docker build -t f1tenth_gym_ros -f Dockerfile .

2. Rocker activate
    . ~/rocker_venv/bin/activate
rocker --nvidia --x11 --volume ../:/sim_ws/src/ -- f1tenth_gym_ros

3. Source path for bash
source /opt/ros/foxy/setup.bash
source install/local_setup.bash

4. launch command
   
ros2 launch f1tenth_gym_ros gym_bridge_launch.py

**Issues Faced**
1. Encountered issues with terminal communication and graphics when set up was done in WSL, so had to switch to a dual-boot setup with Ubuntu.
2. Faced an issue with OpenGL while launching the simulator, which was resolved by running the command `export LIBGL_ALWAYS_SOFTWARE=1`. A similar issue and solution can be found in the mentioned thread https://github.com/ros2/rviz/issues/929 
3. If unable to extend the partition despite having free space, consider using a tool like AOMEI Partition Assistant to manage the partition extension.
4. In case of limited space on an existing Ubuntu installation, use GParted Live on a bootable USB drive to merge existing free partitions.

Execute below command 
export LIBGL_ALWAYS_SOFTWARE=1 

**Creating new package :**
ros2 pkg create --build-type ament_python --node-name my_node my_package

ros2 pkg create --build-type ament_cmake --node-name my_node my_package

**Build**
1. in ros_ws 
colcon build
source install/local_setup.bash
ros2 run project my_node

**Connecting Dockers**
1. use command "docker ps" to get list of existing containers.
2. below command with docker name from list. 
docker exec -it great_ramanujan bash

note : here name of docker is great ramanujan need to change as per your dockername in list.

**Keyboard Teleop command to control car with keyboard**
ros2 run teleop_twist_keyboard teleop_twist_keyboard

