# handeye_calibration_with_depth_camera

Usually, we often use qr code using rgb image to do the calibration. (such as easy_hand_eye calibration: https://github.com/IFL-CAMP/easy_handeye.git)
However, if we use a depth camera, the method above includes many errors, such as intrinsic errors or extrinsic error between depth camera and rgb camera.
Therefore, what we do is to use a calibration plate to solve the problem of AX=XB
![image](https://github.com/pyni/handeye_calibration_with_depth_camera/blob/master/figure/block.png)

To use our package, you should do a 3D printing for the calibration plate and please paste four QR codes on it (These files are listed in the "prepareddata" directory).

# Install (Ubuntu 18.04+ROS melodic)

1.please install easy_handeye (https://github.com/IFL-CAMP/easy_handeye.git), realsense_ros(https://github.com/IntelRealSense/realsense-ros/),in the same src of our packages (Here we also use OpenGR https://storm-irit.github.io/OpenGR/ and you may change the path of opengr in cmakelist of qr_code_icp )

2. catkin_make all the packages

# Preparation of calibration plate
## (Here i have uploaded the prepared data: that is calibblockthird.obj in our package)
One way to get the point clouds of calibration plate is sampling using pcl tools: "pcl_mesh_sampling", the you can trim the mesh points if you want using meshlab.

Then you should move the coordinate system of the calibration plate in the center of the bottom like this, otherwise the pose estimation algorithm may be wrong:

![imageca](https://github.com/pyni/handeye_calibration_with_depth_camera/blob/master/figure/coord.png)


# Calibration in simulation data 

1.roslaunch easy_handeye_demo calibrate.launch

# Calibration in real data and real robot(Here we take UR5):

Firstly, you should launch ur5:

1.roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=192.168.1.102 [reverse_port:=REVERSE_PORT]

2.roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch

3.roslaunch ur5_moveit_config moveit_rviz.launch config:=true

Then you should launch calibration algorithms:

1.roslaunch aruco_ros multi.launch

2.roslaunch realsense2_camera rs_camera.launch filters:=pointcloud align_depth:=true

3.rosrun 3dposedetection calibration

4.roslaunch easy_handeye_demo calibrate_real.launch

(For 3 and 4, each time you want take a sample, you should launch "rosrun 3dposedetection calibration",and take a sample; About 30 samples will get a good calibration result)


# Performance

Here is the performance, the calibration error is with 1~2mm using realsense sr300

![image1](https://github.com/pyni/handeye_calibration_with_depth_camera/blob/master/figure/Screenshot%20from%202020-07-06%2015-08-03.png)

![image2](https://github.com/pyni/handeye_calibration_with_depth_camera/blob/master/figure/Screenshot%20from%202020-07-06%2015-08-04.png)

![image3](https://github.com/pyni/handeye_calibration_with_depth_camera/blob/master/figure/Screenshot%20from%202020-07-06%2015-08-06.png)


# How to improve the accuracy:

method1: https://zhuanlan.zhihu.com/p/157299575

method2: https://github.com/pyni/quick_depth_handeye_calibration_without_calibration_board.git
