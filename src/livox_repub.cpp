#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include "livox_ros_driver/CustomMsg.h"
#include "loam_horizon/common.h"
#include "loam_horizon/mypoint.h"
#include "loam_horizon/lidar_camera_common.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
using namespace std;

ros::Publisher pub_pcl_out1;
uint64_t TO_MERGE_CNT = 1; 
constexpr bool b_dbg_line = false;
std::vector<livox_ros_driver::CustomMsgConstPtr> livox_data;

void getUV(const cv::Mat &matrinxIn, const cv::Mat &matrix_out, float x, float y, float z, float* UV);
void getColor(const cv::Mat &matrinxIn, const cv::Mat &matrix_out, float x, float y, float z, int row, int col, const vector<vector<int>> &color_vector, int* RGB);

string intrinsic_path, extrinsic_path;

// use extrinsic and intrinsic to get the corresponding U and V
void getUV(const cv::Mat &matrix_in, const cv::Mat &matrix_out, float x, float y, float z, float* UV) {
    double matrix3[4][1] = {x, y, z, 1};
    cv::Mat coordinate(4, 1, CV_64F, matrix3);
    
    // calculate the result of u and v
    cv::Mat result = matrix_in*matrix_out*coordinate;
    float u = result.at<double>(0, 0);
    float v = result.at<double>(1, 0);
    float depth = result.at<double>(2, 0);
    
    UV[0] = u / depth;
    UV[1] = v / depth;

}

// get RGB value of the lidar point
void getColor(const cv::Mat &matrix_in, const cv::Mat &matrix_out, float x, float y, float z, int row, int col, const vector<vector<int>> &color_vector, int* RGB) {
    float UV[2] = {0, 0}; 
    getUV(matrix_in, matrix_out, x, y, z, UV);  // get U and V from the x,y,z
    
    int u = int(UV[0]);
    int v = int(UV[1]);

    int32_t index = v*col + u;
    if (index < row*col && index >= 0) {
        RGB[0] = color_vector[index][0];
        RGB[1] = color_vector[index][1];
        RGB[2] = color_vector[index][2];
    }
}

void getParameters() {
    cout << "Get the parameters from the launch file" << endl;
    if (!ros::param::get("intrinsic_path", intrinsic_path)) {
        cout << "Can not get the value of intrinsic_path" << endl;
        exit(1);
    }
    if (!ros::param::get("extrinsic_path", extrinsic_path)) {
        cout << "Can not get the value o以下程序节点中如果想修改launch文件，需要到src/calibration/launch文件夹中找对应的launch文件。f extrinsic_path" << endl;
        exit(1);
    }   
}




void LivoxMsgCbk1(const sensor_msgs::ImageConstPtr& img_msg, const livox_ros_driver::CustomMsgConstPtr& livox_msg_in) 
{

  getParameters();
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat src_img = cv_ptr->image;

  vector<float> intrinsic;
  getIntrinsic(intrinsic_path, intrinsic);
  vector<float> distortion;
  getDistortion(intrinsic_path, distortion);
  vector<float> extrinsic;
  getExtrinsic(extrinsic_path, extrinsic);
    
  // set the intrinsic and extrinsic matrix
  double matrix1[3][3] = {{intrinsic[0], intrinsic[1], intrinsic[2]}, {intrinsic[3], intrinsic[4], intrinsic[5]}, {intrinsic[6], intrinsic[7], intrinsic[8]}}; 
  double matrix2[3][4] = {{extrinsic[0], extrinsic[1], extrinsic[2], extrinsic[3]}, {extrinsic[4], extrinsic[5], extrinsic[6], extrinsic[7]}, {extrinsic[8], extrinsic[9], extrinsic[10], extrinsic[11]}};
    
  // transform into the opencv matrix
  cv::Mat matrix_in(3, 3, CV_64F, matrix1);
  cv::Mat matrix_out(3, 4, CV_64F, matrix2);

	// set intrinsic parameters of the camera
  cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
  camera_matrix.at<double>(0, 0) = intrinsic[0];
  camera_matrix.at<double>(0, 2) = intrinsic[2];
  camera_matrix.at<double>(1, 1) = intrinsic[4];
  camera_matrix.at<double>(1, 2) = intrinsic[5];

	// set radial distortion and tangential distortion
  cv::Mat distortion_coef = cv::Mat::zeros(5, 1, CV_64F);
  distortion_coef.at<double>(0, 0) = distortion[0];
  distortion_coef.at<double>(1, 0) = distortion[1];
  distortion_coef.at<double>(2, 0) = distortion[2];
  distortion_coef.at<double>(3, 0) = distortion[3];
  distortion_coef.at<double>(4, 0) = distortion[4];

  // use intrinsic matrix and distortion matrix to correct the photo first
  cv::Mat view, rview, map1, map2;
  cv::Size imageSize = src_img.size();
  cv::initUndistortRectifyMap(camera_matrix, distortion_coef, cv::Mat(),cv::getOptimalNewCameraMatrix(camera_matrix, distortion_coef, imageSize, 1, imageSize, 0), imageSize, CV_16SC2, map1, map2);
  cv::remap(src_img, src_img, map1, map2, cv::INTER_LINEAR);  // correct the distortion

  int row = src_img.rows;
  int col = src_img.cols;
  // cout << row << endl;
  // cout << col << endl << endl;
  vector<vector<int>> color_vector;
  color_vector.resize(row*col);
  for (unsigned int i = 0; i < color_vector.size(); ++i) 
  {
    color_vector[i].resize(3);
  }
    
  // read photo and get all RGB information into color_vector
    
  for (int v = 0; v < row; ++v) 
  {
    for (int u = 0; u < col; ++u) 
    {
      // for .bmp photo, the 3 channels are BGR
      color_vector[v*col + u][0] = src_img.at<cv::Vec3b>(v, u)[2];
      color_vector[v*col + u][1] = src_img.at<cv::Vec3b>(v, u)[1];
      color_vector[v*col + u][2] = src_img.at<cv::Vec3b>(v, u)[0];
    }
  }

  livox_data.push_back(livox_msg_in);
  uint64_t num = 0;

  pcl::PointCloud<PointType> pcl_in;
  pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
  cloud->is_dense = false;
  cloud->height = 1;
  cloud->width = livox_data[num]->point_num; // get the point number of lidar data
  cloud->points.resize(cloud->width);
  for (size_t j = 0; j < livox_data.size(); j++) {
    auto& livox_msg = livox_data[j];
    auto time_end = livox_msg->points.back().offset_time;
    for (unsigned int i = 0; i < livox_msg->point_num; ++i) {
      PointType pt;
      float x = livox_msg->points[i].x;
      float y = livox_msg->points[i].y;
      float z = livox_msg->points[i].z;
      pt.x = x;
      pt.y = y;
      pt.z = z;
      
      cloud->points[i].x = x;
      cloud->points[i].y = y;
      cloud->points[i].z = z;

      if(x == 0 && y == 0 && z == 0) 
      {
        continue;
      }

      // set the RGB for the cloud point 
      int RGB[3] = {0, 0, 0}; 
      getColor(matrix_in, matrix_out, x, y, z, row, col, color_vector, RGB); 
      // ignore the unexisting point
      if (RGB[0] == 0 && RGB[1] == 0 && RGB[2] == 0) 
      {
        continue;
      }
      pt.r = RGB[0];
      pt.g = RGB[1];
      pt.b = RGB[2];
      cloud->points[i].r = RGB[0];
      cloud->points[i].g = RGB[1];
      cloud->points[i].b = RGB[2];
      
//      if (pt.z < -0.3) continue; // delete some outliers (our Horizon's assembly height is 0.3 meters)
      float s = livox_msg->points[i].offset_time / (float)time_end;
//       ROS_INFO("_s-------- %.6f ",s);
      pt.intensity = livox_msg->points[i].line + s*0.1; // The integer part is line number and the decimal part is timestamp
//      ROS_INFO("intensity-------- %.6f ",pt.intensity);
      pt.curvature = livox_msg->points[i].reflectivity * 0.1;
      // ROS_INFO("pt.curvature-------- %.3f ",pt.curvature);
      pcl_in.push_back(pt);

      cloud->points[i].intensity = livox_msg->points[i].line + s*0.1;
      cloud->points[i].curvature = livox_msg->points[i].reflectivity * 0.1;
    }
  }

  /// timebase 5ms ~ 50000000, so 10 ~ 1ns

  unsigned long timebase_ns = livox_data[0]->timebase;
  ros::Time timestamp;
  timestamp.fromNSec(timebase_ns);

  //   ROS_INFO("livox1 republish %u points at time %f buf size %ld",
  //   pcl_in.size(),
  //           timestamp.toSec(), livox_data.size());

  sensor_msgs::PointCloud2 pcl_ros_msg;
  // pcl::toROSMsg(pcl_in, pcl_ros_msg);
  pcl::toROSMsg(*cloud, pcl_ros_msg);
  pcl_ros_msg.header = livox_msg_in->header;
  pcl_ros_msg.header.stamp.fromNSec(timebase_ns);
  pcl_ros_msg.header.frame_id = "/livox";
  pub_pcl_out1.publish(pcl_ros_msg);
  livox_data.clear();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "livox_repub");
  ros::NodeHandle nh;
  
  ROS_INFO("start livox_repub");

  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/rgb_cam/image_raw", 1);
  message_filters::Subscriber<livox_ros_driver::CustomMsg> lidar_sub(nh, "/livox/lidar/time_sync", 1);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, livox_ros_driver::CustomMsg> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> img_sync(MySyncPolicy(10), image_sub, lidar_sub);
  img_sync.registerCallback(boost::bind(&LivoxMsgCbk1, _1, _2));

  pub_pcl_out1 = nh.advertise<sensor_msgs::PointCloud2>("/livox_pcl0", 1);

  ros::spin();
  }
