# Stereo 3D reconstruction
This project is using stereo camera to recontrust the 3D enviroment. The project didn't add parallel threads yet, but it will be updated in the furture.   
The Code performs well when using Kitti datasets(00, 01, 02). To run the code, you should put your own dataset and result foloder into the project. The code of loading the data should also be adjust(depending where the dataset you put).
  
Before install the project, make sure you have install all Third party libraries. They are:  
PCL(point cloud library)  
Opencv  
Pangolin  
Ceres  
  
After install all libraries, open a terminal in the root dictionary, then, follow the steps to install the project:  
1.mkdir build  
2.cd build  
3.cmake ..  
4.make  
After the project is installed, type ./3D_reconstruct to run the project. 
