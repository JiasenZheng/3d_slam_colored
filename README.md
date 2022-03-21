# 3D SLAM and Point Cloud Colourisation

The package implements 3D SLAM using the Rtab-map ROS package on a Jackal UGV. The Jackal is equipped with a Velodyne lidar and a Realsense camera. A point cloud colorization algorithm and an extrinsic calibration package are also developed in the project. A detailed explanation of the project can be found in my [portfolio post](https://jiasenzheng.github.io/projects/0-slam-and-point-cloud-colourisation). The instructions of calibration can be found in the [README](https://github.com/JiasenZheng/velo2rs_calibration) of the calibration package.

### Dependencies
* Jackal
* tf2_ros
* move_base
* rtabmap_ros
* pcl library
* cv_bridge
* velodyne_pointcloud
* realsnese2_camera
* [velo2rs](https://github.com/JiasenZheng/velo2rs_calibration)

### Jackal setup
Please follow the [instructions](https://github.com/dinvincible98/Jackal_ROS_Noetic_Bringup) in Mingqing Yuan's repository to setup the Jackal UGV in ROS (noetic).

**In addtion to his instruction**
<br>
* Include the Realsense bringup launch file to the jackal_bringup launch file.
    - set the "align_depth" argument to be true; this is because the rtabmap requires to input a depth-registered image
    - set the "filters" to be "pointcloud" this is because we need to use "sensor_msgs/PointCloud2" message for the calibration

* In the jackal_control launch file.
    - set the "joystick" to be true to use a PS4 controller
    - set the "joy_dev" to be "/dev/input/js0," which is the port to receive the PS4 controller signal through Bluetooth

### 3D SLAM
<br>

* The rtabmap-related node and the parameter setups are all include in "3d_odom_rtabmap.launch" file for our configurations
* To launch the 3D slam in simulation, in the terminal, run the following command:
```shell
roslaunch slam gazebo_simulation.launch use_rtab:=true
```
* To run the 3D slam in the real Jackal, follow the steps below:
    - turn on your Jackal and connect the PS4 controller through Bluetooth
    - ssh to your Jackal computer
    - source the jackal setup bash "setup_jackal.bash" under the slam package directory
    - run the 3d slam launch file in the terminal:
    ```shell
    roslaunch slam 3d_slam.launch
    ```
    - in the client (your personal linux pc), source the pc setup bash "setup_pc.bash" under the slam package directory
    - run the interface launch file by:
    ```shell
    roslaunch slam jackal_interface.launch slam:=true
    ```
    - control the Jackal to move around using the PS4 controller and enjoy slamming

### Calibration
To calibrate and obtain the extrinsic parameters between lidar and the camera in the real robot, follow the steps below:

    * turn on your Jackal and place your target object in the front of the robot
    * ssh to your Jackal computer
    * source the jackal setup bash "setup_jackal.bash" under the slam package directory
    * run the Jackal find points launch file in the terminal:
    ```shell
    roslaunch slam jackal_find_points.launch 
    ```
    * in the client (your personal linux pc), source the pc setup bash "setup_pc.bash" under the slam package directory
    * run the interface launch file by:
    ```shell
    roslaunch slam jackal_interface.launch calib:=true
    ```
    * follow the instructions in the [README](https://github.com/JiasenZheng/velo2rs_calibration) of the calibration package to find and update the reference points
    * after collecting and saving all six reference points, run the calibration node to get the extrinsic parameters by:
    ```shell
    rosrun velo2rs calibration
    ```
    * update the static transform in the "jackal_bringup.launch" with the calibrated extrinsic parameters

### Point Cloud Colourization
<br>

* To run it in simulation, follow the steps below:
    - run the simulation environment by:
    ```shell
    roslaunch slam color_cloud_simulation.launch
    ```
    - run the color cloud node by:
    ```shell
    rosrun slam color_cloud
    ```
* To run it in the real robot without SLAM, follow the steps below:
    - turn on your Jackal and connect the PS4 controller through Bluetooth
    - ssh to your Jackal computer
    - source the jackal setup bash "setup_jackal.bash" under the slam package directory
    - run the basic setup of the robot by:
    ```shell
    roslaunch slam color_cloud_real.launch
    ```
    - run the color cloud node by:
    ```shell
    rosrun slam color_cloud_real
    ```
    - in the client (your personal linux pc), source the pc setup bash "setup_pc.bash" under the slam package directory
    - run the interface launch file by:
    ```shell
    roslaunch slam jackal_interface.launch color_cloud:=true
    ```
    - use the PS4 controller to move the robot and visualize the colorized cloud in Rviz
* To run it in the real robot with SLAM, follow the steps below:
    - turn on your Jackal and connect the PS4 controller through Bluetooth
    - ssh to your Jackal computer
    - source the jackal setup bash "setup_jackal.bash" under the slam package directory
    - run the 3d slam launch file in the terminal:
    ```shell
    roslaunch slam 3d_slam.launch
    ```
    - run the color cloud node by:
    ```shell
    rosrun slam color_cloud_real
    ```
    - in the client (your personal linux pc), source the pc setup bash "setup_pc.bash" under the slam package directory
    - run the interface launch file by:
    ```shell
    roslaunch slam jackal_interface.launch slam:=true
    ```
    - control the Jackal to move around using the PS4 controller and enjoy slamming with colorized cloud




    
    




