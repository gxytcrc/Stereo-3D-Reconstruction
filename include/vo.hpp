#ifndef  vo_hpp
#define vo_hpp

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>


#include <iostream>
#include <ctype.h>
#include <algorithm>
#include <iterator>
#include <vector>
#include <ctime>
#include <sstream>
#include <fstream>
#include <string>



#include "feature.h"
#include "utils.h"
#include "visualOdometry.h"
#include "Frame.h"

using namespace cv;
using namespace std;

class vo
{
public:
    
    void insertValue(Mat&Leftimage0, Mat&Rightimage0, Mat&Leftimage1, Mat&Rightimage1, Mat&PreviousPose, Mat&initTrajectory, FeatureSet current_Features, int frameID, vector<float>&camera, float scale);
    void visualOdometry();
    void displayTrajectory();
    vector<float> matrix2angle(Eigen::Matrix3f rotateMatrix);
    Mat trajectory;
    Mat frame_pose;
    FeatureSet currentVOFeatures;

private:
    Mat imageLeft_t0;
    Mat imageRight_t0;
    Mat imageLeft_t1;
    Mat imageRight_t1;
    Mat projMatrl;
    Mat projMatrr;
    int frame_id;
    float frame_time;
    float fps;
    float fx; 
    float fy; 
    float cx; 
    float cy;
    float bf;
    float s;         
};

#endif /* pointCloudMapping */
