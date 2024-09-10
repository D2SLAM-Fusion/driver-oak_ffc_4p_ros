This is the updated version of OAK-FFC-4P ROS1 wrapper.
This driver is designed for [OmniNxt](https://github.com/HKUST-Aerial-Robotics/OmniNxt)


### How to use
```shell
git clone https://github.com/D2SLAM-Fusion/oak_ffc_4p_ros.git
```

 **Compile Docker images**

```shell
cd ./oak_ffc_4p_ros/docker
make
```

**Create Docker container and build the ROS wrapper in container**

```shell
cd ./oak_ffc_4p_ros
./start_docker.sh 1
### Build ROS wrapper
catkin build
```

**Start ROS wrapper Node **

```shell
source ./devel/setup.bash 
roslaunch oak_ffc_4p_ros OV9782.launch 
```



### Configurations

### sharpness_calibration_mode 

```shell
roslaunch oak_ffc_4p_ros OV9782.launch sharpness_calibration_mode:=true
```

e.g 

![image-20240627163035712](https://raw.githubusercontent.com/Peize-Liu/my-images/master/202406271630345.png)

### Normal_mode : send all image together with one topic

```
roslaunch oak_ffc_4p_ros OV9782.launch
```

![image-20240627163209926](https://raw.githubusercontent.com/Peize-Liu/my-images/master/202406271632004.png)

### Tools
We provide python tools to process the rosbag data

```
conda env create -f environment.yml
conda activate oak_ros_tool_env
```
