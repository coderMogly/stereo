/* Author Anirudh Yadav
*/
#include <stereo_loc/stereo_loc.h>
#include <cmath>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <chrono>

using namespace std;
using namespace Eigen;

namespace stereo_loc{
    StereoLoc::StereoLoc(tf2_ros::Buffer& tf): tf_(tf), octree(128.0f),
                         velo_global(NULL), sync(MySyncPolicy(10),imageLeft_sub, imageRight_sub, infoLeft_sub, infoRight_sub), 
                         scale(0.32), is_init(false){
        //loading global pointcloud
        std::string filename = "./global.pcd";
        velo_global = loadPCD(filename);

        if(velo_global->height!=1)ROS_WARN("stereo_loc: the height of velo_global is %d", velo_global->height);

        octree.setInputCloud (velo_global);
        octree.addPointsFromInputCloud ();

        ros::NodeHandle nh;
        imageLeft_sub.subscribe(nh,"/kitti/camera_gray_left/image_raw", 10);
        imageRight_sub.subscribe(nh, "/kitti/camera_gray_right/image_raw", 10);
        infoLeft_sub.subscribe(nh, "/kitti/camera_gray_left/camera_info", 10);
        infoRight_sub.subscribe(nh, "/kitti/camera_gray_right/camera_info", 10);

        sync.registerCallback(boost::bind(&StereoLoc::combinedCallback, this, _1, _2, _3, _4));

        EST_pose = Matrix4d::Identity();
        IN_pose = Matrix4d::Identity();
        update_pose = Matrix4d::Identity();
        
        update_pose(2,3) = 0.8;
        optimized_T = Matrix4d::Identity();
        GT_pose = Matrix4d::Identity();

        base_line = 0.482;
        global_frame_id = "/world";

        base_link_frame_id = "/base_link"
    }



    pcl::PointCloud<pcl::PointXYZ>::Ptr StereoLoc::loadPCD(std::string filename){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ> (filename, *cloud) == -1) //* load the file
        {
            ROS_ERROR ("Couldn't read file test_pcd.pcd \n");
            //            return (-1);
        }
        ROS_INFO("stereo_loc: Loaded %d Points from file %s", cloud->width * cloud->height, filename.c_str());
        

        return cloud;
    }

    void StereoLoc::combinedCallback(const sensor_msgs::Image::ConstPtr& left_image_msg,const sensor_msgs::Image::ConstPtr& right_image_msg,const sensor_msgs::CameraInfo::ConstPtr& left_info_msg,const sensor_msgs::CameraInfo::ConstPtr& right_info_msg){
        ROS_INFO("steroe_loc combinedCallback got a hit");
        
        
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(left_image_msg, sensor_msgs::image_encodings::MONO8);
        left_image = cv_ptr->image;

        if(!is_init){
            is_init = initial_width = left_image.cols;
            first_image = true;
        }
        cv::resize(left_image, left_image, cv::Size(), scale, scale);

        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
        clahe->setClipLimit(5);
        clahe->apply(left_image,left_image);
        P_left = Map<const MatrixXd>(&left_info_msg->P[0], 3, 4);

        cv_ptr = cv_bridge::toCvCopy(right_image_msg, sensor_msgs::image_encodings::MONO8);
        right_image = cv_ptr->image;
        cv::resize(right_image, right_image, cv::Size(), scale, scale);
        clahe->apply(right_image, right_image);
        P_right = Map<const MatrixXd>(&right_info_msg->P[0], 3, 4);

        try
        {
            baselinkTOworld = tf_.lookupTransform(global_frame_id, base_link_frame_id, ros::Time::now(), ros::Duration(0.1));
        }
        catch (tf2::LookupException& ex)
        {
            ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
            return false;
        }
        Eigen::Affine3d e_temp;
        e_temp = tf2::transformTFToEigen(baselinkTOworld);
        GT_pose.matrix() = e_temp.matrix();

        if(!is_init){
            setCamInfo();
        }

        cv::normalize(left_image, left_scaled, 0, 1, CV_MINMAX, CV_32FC1);   
        cv::Scharr(left_scaled, igx_image, CV_32FC1, 1, 0);
        cv::Scharr(left_scaled, igy_image, CV_32FC1, 0, 1);

        disp = cv::Mat::zeros(cv::Size(width, height), CV_16S);
        cv::Ptr<cv::StereoSGBM> sbm;
        sbm = cv::StereoSGBM::create(0,16*5,7);
        sbm->compute(left_image, right_image, disp);

        pcl::PointCloud<pcl::PointXYZ>::Ptr image_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        image_cloud->width    = width;
        image_cloud->height   = height;
        image_cloud->is_dense = false;
        image_cloud->points.resize (image_cloud->width * image_cloud->height);

        int u,v;
        for(size_t i=0; i<width*height;i++)
        {
            u = i%width;
            v = i/width;
            int16_t d = disp.at<int16_t>(v,u);
            if(d==0 || d!=d || d<d_limit){ 
                d = 0;
            }
            depth[i] = K(0,0)*base_line*( 1.0/((float)d/16.0) + d_var/((float)(d/16.0)*(d/16.0)*(d/16.0)) );
            depth_image.at<float>(v,u) = depth[i];

            float igx = igx_image.at<float>(v,u)/32.0f;
            float igy = igy_image.at<float>(v,u)/32.0f;
            float info_nom = sqrt(igx*igx+igy*igy);
            if(!isfinite(info_nom)){
                image_info[i] = 0;
            }else{ 
                image_info[i] = 1000.0f*sqrt(igx*igx+igy*igy);
            }
            if(!first_image){
                ref_container[i] = ref.at<float>(v,u);
                src_container[i] = left_scaled.at<float>(v,u);
                image_gradientX[i] = igx_image.at<float>(v,u)/32.0f;
                image_gradientY[i] = igy_image.at<float>(v,u)/32.0f;
                ref_image_gradientX[i] = ref_igx_image.at<float>(v,u)/32.0f;
                ref_image_gradientY[i] = ref_igy_image.at<float>(v,u)/32.0f;

                igx = ref_igx_image.at<float>(v,u)/32.0f;
                igy = ref_igy_image.at<float>(v,u)/32.0f;
                info_nom = sqrt(igx*igx+igy*igy);
                if(!isfinite(info_nom)){
                    ref_image_info[i] = 0;
                }else{ 
                    ref_image_info[i] = 1000.0f*sqrt(igx*igx+igy*igy);
                }

                ref_depth[i] = ref_depth_image.at<float>(v,u);
                ref_depth_gradientX[i] = ref_dgx_image.at<float>(v,u)/32.0f;
                ref_depth_gradientY[i] = ref_dgy_image.at<float>(v,u)/32.0f;

                igx = ref_dgx_image.at<float>(v,u)/32.0f;
                igy = ref_dgy_image.at<float>(v,u)/32.0f;
                info_nom = sqrt(igx*igx+igy*igy);
                if (!isfinite(info_nom)) {
                    ref_depth_info[i] = 0;
                }else if (info_nom<0.01){
                    ref_depth_info[i] = 0.0;
                }else {
                    ref_depth_info[i] = 1.0/info_nom;
                }
            }
        }
        
        
        cv::Scharr(depth_image, dgx_image, CV_32FC1, 1, 0);
        cv::Scharr(depth_image, dgy_image, CV_32FC1, 0, 1);
        int count_gradient = 0; 
        
        for(size_t i=0; i<width*height;i++)
        {
            u = i%width;
            v = i/width;

            //depth gradient
            depth_gradientX[i] = dgx_image.at<float>(v,u)/32.0f;
            depth_gradientY[i] = dgy_image.at<float>(v,u)/32.0f;

            //depth info
            float info_denom = sqrt(depth_gradientX[i]*depth_gradientX[i]+depth_gradientY[i]*depth_gradientY[i]);
            if (!isfinite(info_denom)){ 
                depth_info[i] = 0;
            }else if (info_denom<0.01){ 
                depth_info[i] = 0.0;
            }else{ 
                depth_info[i] = 1.0/info_denom;
            }
            //cloud plot
            if(isfinite(depth[i])){
                image_cloud->points[i].x = depth[i]/K(0,0)*(u-K(0,2));
                image_cloud->points[i].y = depth[i]/K(1,1)*(v-K(1,2)); 
                image_cloud->points[i].z = depth[i];
            }   
        }
        GT_pose = IN_pose.inverse()*GT_pose;

        if(!first_image){
            
        }
        
        
        
        if(first_image){
            first_image = false;
        }
    
    }

    bool StereoLoc::setCamInfo(){
        float fx = P_left(0,0)*right_image.cols/ancient_width;
        float fy = P_left(1,1)*right_image.cols/ancient_width;        
        float cx = P_left(0,2)*right_image.cols/ancient_width;
        float cy = P_left(1,2)*right_image.cols/ancient_width;

        base_line = -P_right(0,3)/P_left(0,0);// because R = I;

        K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;
        width = left_image.cols;
        height = left_image.rows;

        //Set ref images
        ref_container = new float[width*height];
        src_container = new float[width*height];
        ref_image_gradientX = new float[width*height];
        ref_image_gradientY = new float[width*height];
        image_gradientX = new float[width*height];
        image_gradientY = new float[width*height];
        ref_image_info = new float[width*height]();
        image_info= new float[width*height]();

        depth_image = cv::Mat::zeros(cv::Size(left_image.cols, left_image.rows), CV_32FC1);

        //Set informations
        depth = new float[width*height]();
        depth_gradientX = new float[width*height]();
        depth_gradientY = new float[width*height]();
        depth_info = new float[width*height]();
        ref_depth = new float[width*height]();
        ref_depth_gradientX = new float[width*height]();
        ref_depth_gradientY = new float[width*height]();
        ref_depth_info = new float[width*height]();

        IN_pose = GT_pose;
        EST_pose = Matrix4d::Identity();

        d_var = 0.01;
        d_limit = 50.0;

        matching_thres = K(0,0)*base_line*( 1.0/(d_limit/16.0) + d_var/((float)(d_limit/16.0)*(d_limit/16.0)*(d_limit/16.0)) );
        
        return true;
    }
};
