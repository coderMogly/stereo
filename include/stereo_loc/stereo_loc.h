/* Author: Anirudh Yadav
*/
#include <vector>
#include <string>

#include <ros/ros.h>

#include <math.h>
#include <mathlib/math_lib.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/octree/octree.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree_iterator.h>
#include <pcl/octree/octree_impl.h> 

#include <pcl/correspondence.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>


#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h> 
#include <opencv2/calib3d.hpp>

using namespace std;
using namespace Eigen;


typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> MySyncPolicy;
namespace stereo_loc{
    class StereoLoc{
        public:
            StereoLoc(tf2_ros::Buffer& tf);
            ~StereoLoc(){}
        private:
            pcl::PointCloud<pcl::PointXYZ>::Ptr loadPCD(std::string filename);
            bool setCamInfo();
            void combinedCallback(const sensor_msgs::Image::ConstPtr& left_image_msg,const sensor_msgs::Image::ConstPtr& right_image_msg,const sensor_msgs::CameraInfo::ConstPtr& left_info_msg,const sensor_msgs::CameraInfo::ConstPtr& right_info_msg);



            tf2_ros::Buffer& tf_;
            pcl::PointCloud<pcl::PointXYZ>::Ptr velo_global;
            pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree;
            message_filters::Subscriber<sensor_msgs::Image> imageLeft_sub;
            message_filters::Subscriber<sensor_msgs::Image> imageRight_sub;
            message_filters::Subscriber<sensor_msgs::CameraInfo> infoLeft_sub;
            message_filters::Subscriber<sensor_msgs::CameraInfo> infoRight_sub;
            message_filters::Synchronizer<MySyncPolicy> sync;

            cv::Mat left_image;
            cv::Mat right_image;
            cv::Mat left_scaled;
            cv::Mat ref;
            cv::Mat depth_image;
            cv::Mat ref_depth_image;
            double scale;
            cv::Mat igx_image, igy_image;
            cv::Mat ref_igx_image, ref_igy_image;
            cv::Mat ref_dgx_image, ref_dgy_image;
            cv::Mat dgx_image, dgy_image;
            cv::Mat disp;

            Matrix<double,3,4> P_left;
            Matrix<double,3,4> P_right;

            bool is_init, first_image;
            int initial_width;
            double base_line;
            int width;
            int height;
            Matrix3f K;


            float* ref_container;
            float* src_container;
            float* ref_image_gradientX;
            float* ref_image_gradientY;
            float* image_gradientX;
            float* image_gradientY;
            float* ref_image_info;
            float* image_info;
        
            float* depth;
            float* depth_gradientX;
            float* depth_gradientY;
            float* depth_info;
            float* ref_depth;
            float* ref_depth_gradientX;
            float* ref_depth_gradientY;
            float* ref_depth_info;

            Matrix4d GT_pose;
            Matrix4d IN_pose;
            Matrix4d EST_pose;
            Matrix4d update_pose;
            Matrix4d optimized_T;
            float matching_thres;
            float d_var;
            float d_limit;

            std::string global_frame_id, right_frame_id, left_frame_id, base_link_frame_id;

            geometry_msgs::TransformStamped baselinkTOworld;
            

    };
};