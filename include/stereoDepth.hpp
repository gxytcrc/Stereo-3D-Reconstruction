#ifndef stereoDepth_hpp
#define stereoDepth_hpp

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
#include "elas.h"
using namespace cv;
using namespace std;
using namespace pcl;

class stereoDepth
{
public:
    void insertValue(Mat&LeftImage, Mat&RightImage, vector<float>&camera, float scale);
    void computeDisparity();//compute the Disparity
    void computeDisparityElas();
    void DisparityFilter();
    void computeDepth();//using Disparity comput the depth
    void compute3Dmap();
    Mat depth;    
    Mat display;

    
private:
    Mat Left;
    Mat Right;
    Mat disparity;
    Mat image;
    float s;
    float fx; 
    float fy; 
    float cx;
    float cx2; 
    float cy;
    float baseline;         
};

#endif /* stereoDepth */
