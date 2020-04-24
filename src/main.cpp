#include "pointCloudMapping.hpp"
#include "vo.hpp"
#include <string.h>

using namespace cv;
using namespace std;
using namespace Eigen;

int main(int argc, const char * argv[])
{
    char file_name[1024];
    char fullpath[1024];
    sprintf(file_name, "%s/", argv[1]);
    sprintf(fullpath,"../Dataset/%s",file_name);
    int fileNumber = atoi(argv[2]);
    cout<<"file number is: "<<fileNumber<<endl;
    //initialize-----------------------------------------------------------------------------------------------------------------
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    string dir = fullpath;
    //generate the camera value and scale
    vector<float> camera(6);
    float scale = 1;
    camera[0] = 120; //baseline
    camera[1] = 365.6586; //fx
    camera[2] = 364.4385; //fy
    camera[3] = 373.1548; //cx1
    camera[4] = 373.1548; // cx2
    camera[5] = 240.0983;//cy
    stereoDepth stereoDepth;
    pointCloudMapping pointCloudMapping;
    vo vo;

    //generate the  camera matrix
    Mat camMatrix = (cv::Mat_<float>(3, 3) << camera[1], 0., camera[3], 0., camera[2], camera[5], 0.,  0., 1.);
    //generate the distortion coefficient
    Mat disCoffL =  (cv::Mat_<float>(1, 5) << -0.01587035744820722, -0.00994580061347367, 0.00053454058995379, 0.00096800944875358);
    Mat disCoffR =  (cv::Mat_<float>(1, 5) << -0.01045265670761152, -0.02987853636289300, 0.02748416885012191, -0.01060533367718567);

    vector<vector<double> > EulerData;
    imuData(dir, EulerData);

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

    vector<vector<double> > imuRotation;
    imuData(dir, imuRotation);

    //begain loop------------------------------------------------------------------------------------------------------------
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    for(int i =0; i<fileNumber ; i++){
        //skip some frames
        if(i%2 != 0){
            continue;
        }
        //load the image
        char base_name[256];
        sprintf(base_name,"%00d.png",i);
        string left_img_file_name  = dir+"left/left_" + base_name;
        string right_img_file_name =dir+"right/right_" + base_name;

        Mat image0= imread(left_img_file_name,0);
        Mat image1= imread(right_img_file_name,0);
        Mat image_rgb = imread(right_img_file_name);
        Mat img0, img1, img_rgb;

        //undistort the images
        undistort(image0, img0, camMatrix, disCoffL);
        undistort(image1, img1, camMatrix, disCoffL);
        undistort(image_rgb, img_rgb, camMatrix, disCoffL);
        imshow("undistored",img_rgb);

        cout<<" "<<endl;
        cout<<left_img_file_name<<endl;
        cout<<"Load the 2 correspondent images: "<<endl;
        // cout<<"the first image weight: "<<image0.cols<<endl;
        // cout<<"the first image height: "<<image0.rows<<endl;
        // cout<<"the second image weight: "<<image1.cols<<endl;
        // cout<<"the second image height: "<<image1.rows<<endl;
        cout<<"----------------------------------------"<<endl;
        preProcess(image0, image0);
        preProcess(image1, image1);
        //unevenLightCompensate(image0, 32);
        //unevenLightCompensate(image1, 32);
        //Tracking--------------------------------------------------------------------------------------------------------
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
        if(i ==0){
            previousImg0 = image0;
            previousImg1 = image1;
        }
        else{
            //begain the visual odometry
            vo.insertValue(previousImg0, previousImg1, image0, image1, frame_pose, trajectory, currentVOFeatures, i, camera, scale);
            //vo.preProcess();
            vo.visualOdometry();
            vo.displayTrajectory();
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
            if(distance>1 || keyframeID >20|| sumAngle > 10){
                keyframe = true;
                LastKeyframePose = Pose;
                R_keyframe = R;
                cout<<"----------------//////////////Found a new keyframe!//////////---------------------"<<endl;
            }
        }
        //Mapping--------------------------------------------------------------------------------------------------------
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////    }

        if(keyframe == true){
            //obtain the Dsiparity value
            stereoDepth.insertValue(image0, image1, camera, scale);
            stereoDepth.computeDisparityElas();
            //stereoDepth.DisparityFilter();
            stereoDepth.computeDepth();
            //calculate the point cloud
            pointCloudMapping.insertValue(cloud, stereoDepth.depth, image_rgb, frame_pose, camera, scale);
            pointCloudMapping.initialize3Dmap();
            //filter the pointcloud(if need)
            //pointCloudMapping.pointCloudFilter();
            cloud = pointCloudMapping.outputPointCloud();
            keyframe = false;
            keyframeID = 0;
            viewer.showCloud(cloud);
        }
        if(i ==0){
            waitKey(15000);
        }
        waitKey(1);
    }
    pcl::io::savePLYFile ("../Result/changzhou.ply", *cloud); 
    pointCloudMapping.pointVisuallize();
}
