# Stereo 3D reconstruction
This project is using stereo camera to recontrust the 3D enviroment. There are two alternative method can compute the disparity: SGBM or Libelas. 
The Code performs well when using Kitti datasets(00, 01, 02). To run the code, you should put your own dataset and result foloder into the project. The code of loading the data should also be adjust(depending where the dataset you put).
  
# 3dReconstruct
3D reconstruction system capable of capturing comprehensive dense globally consistentmaps explored using a stereo camera.

# 1. What do I need to build it? #
* Ubuntu
* CMake
* Eigen
* Pangolin
* OpenCV
* Ceres Solver
* PCL

# 2. How do I use it? #
Clone the repository and catkin_make:
```
    git clone https://github.com/gxytcrc/Stereo-3D-Reconstruction.git
    mkdir build
    cd build
    cmake ..
    make
```
Capture a stereo dataset. Launch it as follows:
```
./3dReconstruct folderName frameNumber
```

