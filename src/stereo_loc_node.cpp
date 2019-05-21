/*Author : Anirudh Yadav
*/
#include <stereo_loc/stereo_loc.h>
#include <tf2_ros/transform_listener.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "stereo_loc_node");
  ros::NodeHandle n;
  tf2_ros::Buffer buffer(ros::Duration(10));
  tf2_ros::TransformListener tf(buffer);

  stereo_loc::StereoLoc stereo_loc(buffer);

  //ros::MultiThreadedSpinner s;
  ros::spin();

  return(0);
}
