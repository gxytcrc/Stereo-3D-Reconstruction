#include "pointCloudMapping.hpp"
#include "vo.hpp"
#include <string.h>
using namespace cv;
using namespace std;
using namespace Eigen;

int main()
{
    //initialize-----------------------------------------------------------------------------------------------------------------
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    string dir = "../Dataset/00/";
    //generate the camera value and scale
    vector<float> camera(6);
    float scale = 1;
    camera[0] = 540; //baseline
    camera[1] = 718.856; //fx
    camera[2] = 718.856; //fy
    camera[3] = 607.1928; //cx1
    camera[4] = 607.1928; // cx2
    camera[5] = 185.2157;//cy
    stereoDepth stereoDepth;
    pointCloudMapping pointCloudMapping;
    vo vo;

    //save the point cloud to ply file
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    //initialize the parameter of visual odometry
    Mat frame_pose = cv::Mat::eye(4, 4, CV_64F);
    Mat trajectory = cv::Mat::zeros(600, 1200, CV_8UC3);
    Mat previousImg0;
    Mat previousImg1;
    FeatureSet currentVOFeatures;
    bool keyframe = true;
    int keyframeID = 0;
    Mat Pose = frame_pose.col(3).clone();
    Mat LastKeyframePose = frame_pose.col(3).clone();
    Matrix3f R = Matrix3f::Identity();
    Matrix3f R_keyframe = Matrix3f::Identity();
    pcl::visualization::CloudViewer viewer("viewer");
    //begain loop------------------------------------------------------------------------------------------------------------
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    for(int i =0; i<300; i++){
        char base_name[256];
        sprintf(base_name,"%06d.png",i);
        string left_img_file_name  = dir+"image_0/" + base_name;
        string right_img_file_name =dir+"image_1/" + base_name;

        Mat image0= imread(left_img_file_name,0);
        Mat image1= imread(right_img_file_name,0);
        Mat image_rgb = imread(right_img_file_name);
        cout<<" "<<endl;
        cout<<left_img_file_name<<endl;
        cout<<"Load the 2 correspondent images: "<<endl;
        // cout<<"the first image weight: "<<image0.cols<<endl;
        // cout<<"the first image height: "<<image0.rows<<endl;
        // cout<<"the second image weight: "<<image1.cols<<endl;
        // cout<<"the second image height: "<<image1.rows<<endl;
        cout<<"----------------------------------------"<<endl;
        //Tracking--------------------------------------------------------------------------------------------------------
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
        if(i == 0){
            previousImg0 = image0;
            previousImg1 = image1;
        }
        else{
            vo.insertValue(previousImg0, previousImg1, image0, image1, frame_pose, trajectory, currentVOFeatures, i, camera, scale);
            vo.visualOdometry();
            //vo.displayTrajectory();
            previousImg0 = image0;
            previousImg1 = image1;
            frame_pose = vo.frame_pose;
            currentVOFeatures = vo.currentVOFeatures;
            trajectory = vo.trajectory;
            //calculate the translation vector
            Pose = frame_pose.col(3).clone();
            //calculate the rotation matrix
            for(int row = 0; row<3; row++){
                for(int col = 0; col<3; col++){
                    R(row, col) = frame_pose.at<double>(row, col);
                }
            }
            //find the keyframe----------------------------------------------------------------------------------------------
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            Matrix3f R_mid = R*R_keyframe.inverse();
            vector<float> angle = vo.matrix2angle(R_mid);
            // cout<<"angleX: "<<angle[0]<<endl;
            // cout<<"angleY: "<<angle[1]<<endl;
            // cout<<"angleZ: "<<angle[2]<<endl;
            float sumAngle = abs(angle[0]) + abs(angle[0]) + abs(angle[0]);
            double distance = sqrt(pow(Pose.at<double>(0) - LastKeyframePose.at<double>(0),2) + pow(Pose.at<double>(2) - LastKeyframePose.at<double>(2),2));
            cout<<"distance: "<<distance<<endl;
            keyframeID = keyframeID + 1;
            if(distance>10 || keyframeID >10|| sumAngle > 20){
                keyframe = true;
                LastKeyframePose = Pose;
                R_keyframe = R;
                cout<<"----------------//////////////Found a new keyframe!//////////---------------------"<<endl;
            }
        }
        //Mapping--------------------------------------------------------------------------------------------------------
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
        if(keyframe == true){
            //obtain the Dsiparity value
            stereoDepth.insertValue(image0, image1, camera, scale);
            stereoDepth.computeDisparityElas();
            stereoDepth.computeDepth();
            //calculate the point cloud
            pointCloudMapping.insertValue(cloud, stereoDepth.depth, image_rgb, frame_pose, camera, scale);
            pointCloudMapping.initialize3Dmap();
            pointCloudMapping.pointCloudFilter();
            cloud = pointCloudMapping.outputPointCloud();
            keyframe = false;
            keyframeID = 0;
            viewer.showCloud(cloud);
        }
        if(i == 1){
            waitKey(10000);
        }
        waitKey(1);
    }
    pointCloudMapping.pointVisuallize();
    //pcl::io::savePLYFile ("../Result/pointcloud.ply", *cloud); 
}
