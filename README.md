# handeye_calibration_with_depth_camera

Usually, we often use qr code using rgb image to do the calibration. (such as easy_hand_eye calibration: https://github.com/IFL-CAMP/easy_handeye.git)
However, if we use a depth camera, the method above includes many errors, such as intrinsic errors or extrinsic error between depth camera and rgb camera.
Therefore, what we do is to use a calibration plate to solve the problem of AX=XB
![image](https://github.com/pyni/handeye_calibration_with_depth_camera/blob/master/figure/block.png)

# Performance

![image1](https://github.com/pyni/handeye_calibration_with_depth_camera/blob/master/figure/Screenshot%20from%202020-07-06%2015-08-03.png)

![image2](https://github.com/pyni/handeye_calibration_with_depth_camera/blob/master/figure/Screenshot%20from%202020-07-06%2015-08-04.png)

![image3](https://github.com/pyni/handeye_calibration_with_depth_camera/blob/master/figure/Screenshot%20from%202020-07-06%2015-08-06.png)

