#include "pointCloudMapping.hpp"

void showPointCloud(
    const vector<Vector4d, Eigen::aligned_allocator<Vector4d>> &pointcloud);


void pointCloudMapping::insertValue(pcl::PointCloud<pcl::PointXYZRGB>::Ptr&insertCloud, Mat&depthmap, Mat&image1, Mat&TransformsValue, vector<float>&camera, float scale){
    cout<<"received the depth map, start generate the point cloud"<<endl;
    image = image1;
    resize(image,image,Size(),scale,scale);
    depth = depthmap;
    s = scale;
    baseline =  camera[0]*s;
    fx = camera[1]*s;
    fy = camera[2]*s; 
    cx = camera[3]*s; 
    cy = camera[5]*s;   
    Cloud    = boost::make_shared<pcl::PointCloud<PointXYZRGB>>( );
    GlobalCloud = boost::make_shared<pcl::PointCloud<PointXYZRGB>>( );
    *GlobalCloud = *insertCloud;
    for(int i =0; i<4; i++){
        for(int j=0; j<4; j++){
            if( j ==3 && i != 3){
                T(i, j) = TransformsValue.at<double>(i, j)*1000;
            }
            else{
                T(i, j) = TransformsValue.at<double>(i, j);
            }
        }
    }
    //cout<<"T: "<<T<<endl;
}

void pointCloudMapping::initialize3Dmap(){
    // cloud->width = depth.cols*depth.rows;
    // cloud->height = 1;
    // cloud->points.resize (cloud->width * cloud->height);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>); 
    for(int i=0;i<depth.rows;i++) {
        for(int j =0; j<depth.cols; j++){

            pcl::PointXYZRGB point;

            double u0= j;
            double v0= i;
            double du0=u0-cx;
            double dv0=v0-cy;
            double x0=du0/fx;
            double y0=dv0/fx;

            float Dep;
            Dep = depth.at<float>(i,j);
            if(i>100){
                if(Dep>3000||Dep<1){
                    continue;
                }
            }
            else{
                if(Dep>6000||Dep<1){
                    continue;
                }
            }
            float X=x0* Dep;
            float Y=y0 * Dep;
            float Z=Dep;
            
            int number = i*depth.cols+j;
            point.x = X;
            point.y = Y;
            point.z = Z;
            
            // cout<<"X: "<<X<<endl;
            // cout<<"Y: "<<Y<<endl;
            // cout<<"Z: "<<Z<<endl; 

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
            point.rgb = *reinterpret_cast<float*>(&rgb);

            temp->push_back(point);
        }
    }
    transformPointCloud (*temp, *temp, T);
    *Cloud = *temp;
    cout<<"The original point cloud has: "<<Cloud->points.size()<<" points"<<endl;
}

void pointCloudMapping::initialize3DmapSemi(){
    // cloud->width = depth.cols*depth.rows;
    // cloud->height = 1;
    // cloud->points.resize (cloud->width * cloud->height);
    double init_sigma = 1.6;
    double k = 1.26;
    vector<double> sigma_current;
    sigma_current.resize(5);
    for (int level = 1; level < 6; level++) {
        sigma_current[level - 1] = sqrt((pow(k, level) * init_sigma) * (pow(k, level) * init_sigma) - (pow(k, level - 1) * init_sigma) * (pow(k, level - 1) * init_sigma));
    }
    Mat img_gaussian1, img_gaussian2;
    Mat img_DoG;
    GaussianBlur(image, img_gaussian1, Size(5, 5), sigma_current[0], 0);
    GaussianBlur(img_gaussian1, img_gaussian2, Size(5, 5), sigma_current[1], 0);
    img_DoG = img_gaussian1 - image;

    Mat disp8U = Mat(img_DoG.rows, img_DoG.cols, CV_8UC1);
    normalize(img_DoG, disp8U, 0, 255, NORM_MINMAX, CV_8UC1);
    imshow("DoG", disp8U);
    //cout<<img_DoG<<endl;
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>); 
    for(int i=0;i<depth.rows;i++) {
        for(int j =0; j<depth.cols; j++){

            pcl::PointXYZRGB point;

            double u0= j;
            double v0= i;
            double du0=u0-cx;
            double dv0=v0-cy;
            double x0=du0/fx;
            double y0=dv0/fx;

            float Dep;
            Dep = depth.at<float>(i,j);
            if(Dep>2500||Dep<1){
                continue;
            }
            disp8U.convertTo(disp8U, CV_32F, 1);
            bool judge = false;
            for(int row = -3; row<4; row++){
                for(int col = -3; col<4; col++){
                    if( i>5&& j>5&& i<depth.rows-5&& j<depth.cols-5){
                        float difference = disp8U.at<float>(i+row, j+col);
                        if(difference >50){
                            judge = true;
                        }
                    }
                }
            }
            if (judge == false){
                continue;
            }
            float X=x0* Dep;
            float Y=y0 * Dep;
            float Z=Dep;
            
            int number = i*depth.cols+j;
            point.x = X;
            point.y = Y;
            point.z = Z;
            
            // cout<<"X: "<<X<<endl;
            // cout<<"Y: "<<Y<<endl;
            // cout<<"Z: "<<Z<<endl; 

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
            point.rgb = *reinterpret_cast<float*>(&rgb);

            temp->push_back(point);
        }
    }
    transformPointCloud (*temp, *temp, T);
    *Cloud = *temp;
    cout<<"The original point cloud has: "<<Cloud->points.size()<<" points"<<endl;
}

void pointCloudMapping::pointFusion(){
    
    
}
void pointCloudMapping::pointCloudFilter(){

    pcl::VoxelGrid<pcl::PointXYZRGB> downSampled;  
    downSampled.setInputCloud (Cloud);            
    downSampled.setLeafSize (100.0f, 100.0f, 100.0f);  
    downSampled.setDownsampleAllData(true);
    downSampled.filter (*Cloud);
              
    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> pcFilter;  
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);    
    pcFilter.setInputCloud(Cloud);             
    pcFilter.setRadiusSearch(200);        
    pcFilter.setMinNeighborsInRadius(4);      
    pcFilter.filter(*Cloud);       

    cout<<"The filtered point cloud has: "<<Cloud->points.size()<<" points"<<endl;
        
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudMapping::outputPointCloud()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>); 
    *temp = *GlobalCloud+*Cloud;
    return temp;
}

void showPointCloud(const vector<Vector4d, Eigen::aligned_allocator<Vector4d>> &pointcloud) {

    if (pointcloud.empty()) {
        cerr << "Point cloud is empty!" << endl;
        return;
    }
    //创建一个窗口
    pangolin::CreateWindowAndBind("Point Cloud Viewer_1", 1024, 768);
    //启动深度测试
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Define Projection and initial ModelView matrix
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
        .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false) {
        // Clear screen and active view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        pangolin::glDrawAxis(3);
        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glPointSize(2);
        glBegin(GL_POINTS);
        for (auto &p: pointcloud) {
            glColor3f(p[3], p[3], p[3]);
            glVertex3d(p[0], p[1], p[2]);
        }
        glEnd();
        // Swap frames and Process Events
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
    return;
}

void pointCloudMapping::pointVisuallize(){
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>); 
    // *temp = *Cloud;
    // //cout<<" has: "<<temp->points.size()<<" points"<<endl;
    // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("point cloud"));
    // viewer->setBackgroundColor(0, 0, 0);
    // pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(temp);
    // viewer->addPointCloud<pcl::PointXYZRGB>(temp, rgb, "Point cloud");
    // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Point cloud");
    // viewer->addCoordinateSystem(100.0);
    // viewer->initCameraParameters();
    // viewer->setCameraPosition(0, 0, 0,    0, 0, 1,   0, -1, 0);
    // while(!viewer->wasStopped()){
    //     viewer->spinOnce(100);
    // }
    // pcl::io::savePLYFile ("../Result/pointcloud.ply", *Cloud);
    vector<Vector4d, Eigen::aligned_allocator<Vector4d>> pointcloud;
    for(int i = 0; i<GlobalCloud->points.size(); i++){
            int r = GlobalCloud->points[i].r;
            int g = GlobalCloud->points[i].g;
            int b = GlobalCloud->points[i].b;
            Eigen::Vector4d point1(0, 0, 0, r/ 255.0); // 前三维为xyz,第四维为颜色
            point1[0] = double(GlobalCloud->points[i].x)/1000.0;
            //cout << "X" << " " << point1[0]  << endl;
            point1[1] = double(GlobalCloud->points[i].y)/1000.0;
            //cout << "Y" << " " << point1[1] << endl;
            point1[2] = double(GlobalCloud->points[i].z)/1000.0;
            //cout << "Z" << " " << point1[2] << endl;
            pointcloud.push_back(point1);
    }
    showPointCloud(pointcloud);
}
