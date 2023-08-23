# StitchPoints

This is a simple Realization of 3D Reconstruction Based on Line Laser. 

Due to the use of low-precision industrial cameras and poor centerline extraction algorithms, and the reconstruction accuracy is not high.

System accuracy depends on calibration accuracy, centerline extraction accuracy and transformation relationship solution accuracy.

In the experiment, aruco code is used to solve the transformation relationship, and HIKVision MV-CA060 industrial camera is used to collect laser lines.

## Dependency

*   OpenCV for math method and calibration

*   PCL for visualization

## Usage

*   MSVC

*   CMake

*   renew the string ( laser images and the corresponding origin images ) in the `main.cpp` .

*   Please attention, this method is a offline 3D-Reconstruction method, and i do not written for real-time acquisition.

