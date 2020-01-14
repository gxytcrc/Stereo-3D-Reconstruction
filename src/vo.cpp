#include "vo.hpp"

void vo::insertValue(Mat&Leftimage0, Mat&Rightimage0, Mat&Leftimage1, Mat&Rightimage1, Mat&PreviousPose, Mat&initTrajectory, FeatureSet current_Features, int frameID, vector<float>&camera, float scale){
    //load image
    frame_id = frameID;
    imageLeft_t0 = Leftimage0;
    imageRight_t0 = Rightimage0;
    imageLeft_t1 = Leftimage1;
    imageRight_t1 = Rightimage1;
    //load the camera parameter
    s = scale;
    bf =  -386.1448*s;
    fx = camera[1]*s;
    fy = camera[2]*s; 
    cx = camera[3]*s; 
    cy = camera[5]*s;   
    
    projMatrl = (cv::Mat_<float>(3, 4) << fx, 0., cx, 0., 0., fy, cy, 0., 0,  0., 1., 0.);
    projMatrr = (cv::Mat_<float>(3, 4) << fx, 0., cx, bf, 0., fy, cy, 0., 0,  0., 1., 0.);

    frame_pose = PreviousPose;
    currentVOFeatures = current_Features;
    trajectory = initTrajectory;
}

void vo::visualOdometry(){

    clock_t t_a, t_b;
    t_a = clock();

    std::vector<cv::Point2f> pointsLeft_t0, pointsRight_t0, pointsLeft_t1, pointsRight_t1;
    matchingFeatures( imageLeft_t0, imageRight_t0,
                    imageLeft_t1, imageRight_t1, 
                    currentVOFeatures,
                    pointsLeft_t0, 
                    pointsRight_t0, 
                    pointsLeft_t1, 
                    pointsRight_t1);  
    
     // Triangulate 3D Points
    cv::Mat points3D_t0, points4D_t0;
    cv::triangulatePoints( projMatrl,  projMatrr,  pointsLeft_t0,  pointsRight_t0,  points4D_t0);
    cv::convertPointsFromHomogeneous(points4D_t0.t(), points3D_t0);

    cv::Mat points3D_t1, points4D_t1;
    cv::triangulatePoints( projMatrl,  projMatrr,  pointsLeft_t1,  pointsRight_t1,  points4D_t1);
    cv::convertPointsFromHomogeneous(points4D_t1.t(), points3D_t1);

    // Tracking transfomation
    // ---------------------
    cv::Mat rotation = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat translation = cv::Mat::zeros(3, 1, CV_64F);

    trackingFrame2Frame(projMatrl, projMatrr, pointsLeft_t0, pointsLeft_t1, points3D_t0, rotation, translation, false);
    displayTracking(imageLeft_t1, pointsLeft_t0, pointsLeft_t1);

    cv::Vec3f rotation_euler = rotationMatrixToEulerAngles(rotation);


    cv::Mat rigid_body_transformation;

    if(abs(rotation_euler[1])<1 && abs(rotation_euler[0])<1 && abs(rotation_euler[2])<1)
    {
        integrateOdometryStereo(frame_id, rigid_body_transformation, frame_pose, rotation, translation);

    } else {

        std::cout << "Too large rotation"  << std::endl;
    }
    t_b = clock();
    frame_time = 1000*(double)(t_b-t_a)/CLOCKS_PER_SEC;
    //std::cout << "frame_pose" << frame_pose << std::endl;
}

void vo::displayTrajectory(){
    fps = 1000/frame_time;
    cout << "[Info] frame times (ms): " << frame_time << endl;
    cout << "[Info] FPS: " << fps << endl;
    cv::Mat xyz = frame_pose.col(3).clone();
    display(frame_id, trajectory, xyz, fps);
}

vector<float> vo::matrix2angle(Eigen::Matrix3f rotateMatrix)
{
	float sy = (float)sqrt(rotateMatrix(0,0) * rotateMatrix(0,0) + rotateMatrix(1,0)*rotateMatrix(1,0));
	bool singular = sy < 1e-6; // If
	float x, y, z;
	if (!singular)
	{
		x = (float)atan2(rotateMatrix(2,1), rotateMatrix(2,2));
		y = (float)atan2(-rotateMatrix(2,0), sy);
		z = (float)atan2(rotateMatrix(1, 0), rotateMatrix(0, 0));
	}
	else
	{
		x = (float)atan2(-rotateMatrix(1, 2), rotateMatrix(1, 1));
		y = (float)atan2(-rotateMatrix(2, 0), sy);
		z = 0;
	}
	vector<float> i;
	i.push_back((float)(x * (180.0f / M_PI)));
	i.push_back((float)(y * (180.0f / M_PI)));
	i.push_back((float)(z * (180.0f / M_PI)));
	return i;
}