#include "stereoDepth.hpp"

void stereoDepth::insertValue(Mat&LeftImage, Mat&RightImage,vector<float>&camera, float scale){
    cout<<"received the stereo image start compute the depth map"<<endl;
    resize(LeftImage,Left,Size(),scale,scale);
    resize(RightImage,Right,Size(),scale,scale);
    s = scale;
    baseline =  camera[0]*s;
    fx = camera[1]*s;
    fy = camera[2]*s; 
    cx = camera[3]*s; 
    cx2 = camera[4]*s; 
    cy = camera[5]*s;    
}


void stereoDepth::computeDisparity(){

    Ptr<cv::StereoSGBM> sgbm = StereoSGBM::create(0, 16, 3);

    sgbm->setPreFilterCap(63);
    int sgbmWinSize =  5;
    int NumDisparities = 96;
    int UniquenessRatio = 6;
    sgbm->setBlockSize(sgbmWinSize);
    int cn = Left.channels();

    sgbm->setP1(8 * cn*sgbmWinSize*sgbmWinSize);
    sgbm->setP2(32 * cn*sgbmWinSize*sgbmWinSize);
    sgbm->setMinDisparity(0);
    sgbm->setNumDisparities(NumDisparities);
    sgbm->setUniquenessRatio(UniquenessRatio);
    sgbm->setSpeckleWindowSize(50);
    sgbm->setSpeckleRange(16);
    sgbm->setDisp12MaxDiff(1);
    sgbm->setMode(StereoSGBM::MODE_HH);

    Mat disparity_sgbm;
    sgbm->compute(Left, Right, disparity_sgbm);
    disparity_sgbm.convertTo(disparity, CV_32F, 1.0/16.0f);
    cout<<"the first disparity weight: "<<disparity.cols<<endl;
    cout<<"the first disparity height: "<<disparity.rows<<endl;

    //cout<<disparity.row(1)<<endl;
    // process the filter
    

    //normalize the disparity for visuallzation
    Mat disp8U = Mat(disparity.rows, disparity.cols, CV_8UC1);
    normalize(disparity, disp8U, 0, 255, NORM_MINMAX, CV_8UC1);
    imshow("disparty", disp8U);
}

void stereoDepth::computeDisparityElas(){

    const Size imsize = Left.size();
    const int32_t dims[3] = {imsize.width, imsize.height, imsize.width};
    Mat leftdpf = Mat::zeros(imsize, CV_32F);
    Mat rightdpf = Mat::zeros(imsize, CV_32F);

    Elas::parameters param(Elas::ROBOTICS);
    param.postprocess_only_left = true;
    //param.filter_adaptive_mean = true;
    Elas elas(param);
    elas.process(Left.data, Right.data, leftdpf.ptr<float>(0), rightdpf.ptr<float>(0), dims);
    leftdpf.convertTo(disparity, CV_32F, 1.);
    Mat disp8U = Mat(disparity.rows, disparity.cols, CV_8UC1);
    normalize(disparity, disp8U, 0, 255, NORM_MINMAX, CV_8UC1);
    imshow("disparty", disp8U);
    //cout<<disparity.row(1)<<endl;

}

void stereoDepth::DisparityFilter()
{
    const int width = disparity.cols;
    const int height = disparity.rows;
    float* data = (float*)disparity.data;
    cv::Mat integralMap = cv::Mat::zeros(height, width, CV_64F);
    cv::Mat ptsMap = cv::Mat::zeros(height, width, CV_32S);
    double* integral = (double*)integralMap.data;
    int* ptsIntegral = (int*)ptsMap.data;
    memset(integral, 0, sizeof(double) * width * height);
    memset(ptsIntegral, 0, sizeof(int) * width * height);
    for (int i = 0; i < height; ++i)
    {
        int id1 = i * width;
        for (int j = 0; j < width; ++j)
        {
            int id2 = id1 + j;
            if (data[id2] > 1e-3)
            {
                integral[id2] = data[id2];
                ptsIntegral[id2] = 1;
            }
        }
    }
    // 积分区间
    for (int i = 0; i < height; ++i)
    {
        int id1 = i * width;
        for (int j = 1; j < width; ++j)
        {
            int id2 = id1 + j;
            integral[id2] += integral[id2 - 1];
            ptsIntegral[id2] += ptsIntegral[id2 - 1];
        }
    }
    for (int i = 1; i < height; ++i)
    {
        int id1 = i * width;
        for (int j = 0; j < width; ++j)
        {
            int id2 = id1 + j;
            integral[id2] += integral[id2 - width];
            ptsIntegral[id2] += ptsIntegral[id2 - width];
        }
    }
    //filter window size
    int wnd;
    double dWnd = 8;
    while (dWnd > 1)
    {
        wnd = int(dWnd);
        dWnd /= 2;
        for (int i = 0; i < height; ++i)
        {
            int id1 = i * width;
            for (int j = 0; j < width; ++j)
            {
                int id2 = id1 + j;
                int left = j - wnd - 1;
                int right = j + wnd;
                int top = i - wnd - 1;
                int bot = i + wnd;
                left = max(0, left);
                right = min(right, width - 1);
                top = max(0, top);
                bot = min(bot, height - 1);
                int dx = right - left;
                int dy = (bot - top) * width;
                int idLeftTop = top * width + left;
                int idRightTop = idLeftTop + dx;
                int idLeftBot = idLeftTop + dy;
                int idRightBot = idLeftBot + dx;
                int ptsCnt = ptsIntegral[idRightBot] + ptsIntegral[idLeftTop] - (ptsIntegral[idLeftBot] + ptsIntegral[idRightTop]);
                double sumGray = integral[idRightBot] + integral[idLeftTop] - (integral[idLeftBot] + integral[idRightTop]);
                if (ptsCnt <= 0)
                {
                    continue;
                }
                data[id2] = float(sumGray / ptsCnt);
            }
        }
        int s = wnd / 2 * 2 + 1;
        if (s > 201)
        {
            s = 201;
        }
        cv::GaussianBlur(disparity, disparity, cv::Size(s, s), s, s);
    }
        Mat disp8U = Mat(disparity.rows, disparity.cols, CV_8UC1);
        normalize(disparity, disp8U, 0, 255, NORM_MINMAX, CV_8UC1);
        imshow("disparty_filtered", disp8U);
}

void stereoDepth::computeDepth(){
    Mat temp = disparity;
    temp = 1/temp;
    depth = (fx*baseline)*temp;
    depth.convertTo(depth, CV_32F, 1);
    for(int i = 0; i<depth.rows; i++){
        for(int j =0; j<depth.cols; j++){
            if(depth.at<float>(i, j) < 0 || depth.at<float>(i, j)> 100000 ){
                depth.at<float>(i, j) = 0;
            }
        }
    }
    Mat inverseDepth = 1/depth;
    Mat disp8U = Mat(depth.rows, depth.cols, CV_8UC1);
    normalize(inverseDepth, disp8U, 0, 255, NORM_MINMAX, CV_8UC1);
    //imshow("depthmap", disp8U);
    display = disp8U;
    cout<<"depth map computation finished"<<endl;
    cout<<" "<<endl;
    //cout<<"depth row: "<<image.rows<<endl;
    //cout<<depth.row(1)<<endl;
}

void stereoDepth::compute3Dmap(){

    image = imread("../Dataset/stereo/im0.png");
    resize(image,image,Size(),0.5,0.5);


	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud->width = depth.cols*depth.rows;
    cloud->height = 1;
    cloud->points.resize (cloud->width * cloud->height);

    for(int i=0;i<depth.rows;i++) {
        for(int j =0; j<depth.cols; j++){
            double u0= j;
            double v0= i;
            double du0=u0-cx;
            double dv0=v0-cy;
            double x0=du0/fx;
            double y0=dv0/fx;

            float Dep;
            Dep = depth.at<float>(i,j);
            if(Dep>20000){
                Dep = 0;
            }

            float X=x0* Dep;
            float Y=y0 * Dep;
            float Z=Dep;
            
            int number = i*depth.cols+j;
            cloud->points[number].x = X;
            cloud->points[number].y = Y;
            cloud->points[number].z = Z;

            int b =  image.at<Vec3b>(i, j)[0];
            int g=  image.at<Vec3b>(i, j)[1];
            int r =  image.at<Vec3b>(i, j)[2];

            unsigned char R= r;
            unsigned char G= g;
            unsigned char B= b;

            uint8_t rp = r;  
            uint8_t gp = g; 
            uint8_t bp = b;
            uint32_t rgb = ((uint32_t )rp<<16 | (uint32_t )gp<<8 |  (uint32_t )bp);
            cloud->points[number].rgb = *reinterpret_cast<float*>(&rgb);
        }
    }
    pcl::io::savePLYFile ("../Result/pointcloud.ply", *cloud); 
}