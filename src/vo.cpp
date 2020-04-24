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
    bf =  -camera[0]*camera[1]*0.001*s*s;
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



//BA cost function
struct cost_function_define
{
  cost_function_define(Point3f p1,Point2f p2):_p1(p1),_p2(p2){}
  template<typename T>
  bool operator()(const T* const cereRotation, const T* const cereTranslation, T* residual)const
  {
    T p_1[3];
    T p_2[3];
    p_1[0]=T(_p1.x);
    p_1[1]=T(_p1.y);
    p_1[2]=T(_p1.z);

    ceres::AngleAxisRotatePoint(cereRotation,p_1,p_2);
    
    p_2[0]=p_2[0]+cereTranslation[0];
    p_2[1]=p_2[1]+cereTranslation[1];
    p_2[2]=p_2[2]+cereTranslation[2];

    const T x0=p_1[0]/p_1[2];
    const T y0=p_1[1]/p_1[2];

    const T u0=x0*365.6586+373.1548; // x0*fx + cx
    const T v0=y0*364.4385+240.0983; // y0*fy + cy



    const T x=p_2[0]/p_2[2];
    const T y=p_2[1]/p_2[2];

    const T u=x*365.6586+373.1548; // x0*fx + cx
    const T v=y*364.4385+240.0983; // y0*fy + cy

    const T u1=T(_p2.x);
    const T v1=T(_p2.y);

    residual[0]=u-u1;
    residual[1]=v-v1;
    // cout<<"mid t: "<<cereTranslation[0]<<endl<<cereTranslation[1]<<endl<<cereTranslation[2]<<endl;
    // cout<<"original u: "<<u0<<endl;
    // cout<<"old u: "<<u1<<endl;
    // cout<<"new u"<<u<<endl;
    return true;
  }
   Point3f _p1;
   Point2f _p2;
};



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
    cv::Mat inliers;
    trackingFrame2Frame(projMatrl, projMatrr, pointsLeft_t0, pointsLeft_t1, points3D_t0, rotation, translation, inliers, false);
    displayTracking(imageLeft_t1, pointsLeft_t0, pointsLeft_t1);
    //cout<<"inlier: "<<inliers<<endl;

    cv::Vec3f rotation_euler = rotationMatrixToEulerAngles(rotation);


    cv::Mat rigid_body_transformation;

    cv::Point3f p1;
    cv::Point2f p2;
    double  cereRotation[3];
    double  cereTranslation[3];

    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
    Rodrigues(rotation, rvec);
    //set initial value
    cereRotation[0]=rvec.at<double>(0,0);
    cereRotation[1]=rvec.at<double>(0,1);
    cereRotation[2]=rvec.at<double>(0,2);
    cereTranslation[0] = translation.at<double>(0,0);
    cereTranslation[1] = translation.at<double>(0,1);
    cereTranslation[2] = translation.at<double>(0,2);
    cout<<"old R = "<<rvec<<endl;
    cout<<"old t =" <<translation<<endl;

    //Bundle adjustment 
    ceres::Problem problem;
    for(int i =0; i<inliers.rows; i++){
        int PointID = inliers.at<int>(i, 0);
        //cout<<"ID: "<<PointID<<endl;
        p1.x = points3D_t0.at<float>(PointID, 0);
        p1.y = points3D_t0.at<float>(PointID, 1);
        p1.z = points3D_t0.at<float>(PointID, 2);
        
        p2 = pointsLeft_t1[PointID];
        ceres::CostFunction* costfunction=new ceres::AutoDiffCostFunction<cost_function_define,2,3,3>(new cost_function_define(p1,p2));
        problem.AddResidualBlock(costfunction,NULL,cereRotation,cereTranslation);

    }
    ceres::Solver::Options option;
    option.linear_solver_type=ceres::DENSE_SCHUR;//DENSE_SCHUR
    option.minimizer_progress_to_stdout=true;
    ceres::Solver::Summary summary;
    ceres::Solve(option,&problem,&summary);
    cout<<summary.BriefReport()<<endl;

    Mat Rotaion_vector=(Mat_<double>(3,1)<<cereRotation[0],cereRotation[1],cereRotation[2]);
    Mat translation_vector=(Mat_<double>(3,1)<<cereTranslation[0],cereTranslation[1],cereTranslation[2]);
    Mat Rotation_matrix;
    Rodrigues(Rotaion_vector,Rotation_matrix);
    cout<<"new R = "<<Rotaion_vector<<endl; 
    cout<<"new t =" <<translation_vector<<endl;
    

    // bool ifBA = true;
    // if(ifBA == true){
    //     for()
    // }


    if(abs(rotation_euler[1])<1 && abs(rotation_euler[0])<1 && abs(rotation_euler[2])<1)
    {
        integrateOdometryStereo(frame_id, rigid_body_transformation, frame_pose, Rotation_matrix, translation_vector);

    } else {
        
        //integrateOdometryStereo(frame_id, rigid_body_transformation, frame_pose, rotation, translation);
        std::cout << "Too large rotation"  << std::endl;
    }
    t_b = clock();
    frame_time = 1000*(double)(t_b-t_a)/CLOCKS_PER_SEC;
    std::cout << "frame_pose" << rotation_euler << std::endl;
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


void preProcess(Mat&image0, Mat&image1){
    Mat kernel=(Mat_<char>(3,3)<<0,-1,0,
                        -1,5,-1,
                        0,-1,0);	
    filter2D(image0, image1, image0.depth(),kernel);
}

void unevenLightCompensate(Mat &image, int blockSize)
{
	if (image.channels() == 3) cvtColor(image, image, 7);
	double average = mean(image)[0];
	int rows_new = ceil(double(image.rows) / double(blockSize));
	int cols_new = ceil(double(image.cols) / double(blockSize));
	Mat blockImage;
	blockImage = Mat::zeros(rows_new, cols_new, CV_32FC1);
	for (int i = 0; i < rows_new; i++)
	{
		for (int j = 0; j < cols_new; j++)
		{
			int rowmin = i*blockSize;
			int rowmax = (i + 1)*blockSize;
			if (rowmax > image.rows) rowmax = image.rows;
			int colmin = j*blockSize;
			int colmax = (j + 1)*blockSize;
			if (colmax > image.cols) colmax = image.cols;
			Mat imageROI = image(Range(rowmin, rowmax), Range(colmin, colmax));
			double temaver = mean(imageROI)[0];
			blockImage.at<float>(i, j) = temaver;
		}
	}
	blockImage = blockImage - average;
	Mat blockImage2;
	resize(blockImage, blockImage2, image.size(), (0, 0), (0, 0), INTER_CUBIC);
	Mat image2;
	image.convertTo(image2, CV_32FC1);
	Mat dst = image2 - blockImage2;
	dst.convertTo(image, CV_8UC1);
}

void imuData(string&dir, vector<vector<double> >&EularRotationData){

    int number = 15046;
    double a[number];
    double b[number];
    double c[number];
    double d[number];
    double e[number];
    double f[number];
    double g[number];
    double h[number];
    double k[number];
    string txtname = dir+"left/test.txt";
    ifstream myfile(txtname);
    int N;  
    myfile>>N;
    vector<double> Rotation(4, 0);
    EularRotationData.push_back(Rotation);
    int count = 0;
    for(int i=0;i<number;i++) 
    {
        myfile>>a[i]>>b[i]>>c[i]>>d[i]>>e[i]>>f[i]>>g[i]>>h[i]>>k[i];
        //cout<<a[i]<<" "<<b[i]<<" "<<c[i]<<" "<<d[i]<<" "<<e[i]<<" "<<f[i]<<" "<<g[i]<<" "<<h[i]<<" "<<k[i]<<" "<<endl;
       // cout<<" "<<endl;
        if(k[i] != 0){
            EularRotationData.push_back(Rotation);
            Rotation[0] = 0;  Rotation[1] = 0; Rotation[2] = 0; Rotation[3] = k[i];
            count += 1;
        }
        Rotation[0] += (e[i]/200)*(3.14159/180);  Rotation[1] += f[i]/200*(3.14159/180); Rotation[2] += g[i]/200*(3.14159/180);
    } 
    myfile.close();
    cout<<"frame size: "<<EularRotationData.size()<<endl;
    
}