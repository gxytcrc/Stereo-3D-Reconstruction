#ifndef pointCloudMapping_hpp
#define pointCloudMapping_hpp

#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include  "opencv2/flann.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>  
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <iostream>  
#include <stdio.h>
#include "stereoDepth.hpp"
#include <boost/make_shared.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <eigen3/Eigen/Dense>
#include <pangolin/pangolin.h>


using namespace cv;
using namespace std;
using namespace pcl;
using namespace Eigen;

class pointCloudMapping
{
public:
    
    void insertValue(pcl::PointCloud<pcl::PointXYZRGB>::Ptr&insertCloud, Mat&depthmap, Mat&image1, Mat&TransformsValue, vector<float>&camera, float scale);
    void initialize3Dmap();
    void pointCloudFilter();
    void pointFusion();
    void pointVisuallize();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputPointCloud();
    
private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr GlobalCloud;
    Matrix4f T = Matrix4f::Identity();
    Mat depth;
    Mat image;
    float fx; 
    float fy; 
    float cx; 
    float cy;
    float baseline;
    float s;         
};

#endif /* pointCloudMapping */
