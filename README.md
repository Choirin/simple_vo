# simple visual odometry sample code using opencv sfm apis
Estimate the relative pose between two frames using recoverPose.
Key frames insertion and pose estimation using PnP from key points are implemented but not worked correctly.

## installation
### dependencies
OpenCV 3.2 or later

### build
```
$ mkdir build
$ cd build
$ cmake ..
$ make
```

### run
Modify path to kitti dataset path in simple_vo.cpp  `sTemplate`, and build.
If you use other dataset, change camera matrix and distortion in decomposer.cpp also.
And just
```
$ ./build/simple_vo
```

The estimated trajectories would be shown in a window.
