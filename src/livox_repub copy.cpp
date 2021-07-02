#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include "livox_ros_driver/CustomMsg.h"
#include "loam_horizon/common.h"
#include "loam_horizon/mypoint.h"

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

ros::Publisher pub_pcl_out0, pub_pcl_out1;
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
    if (!ros::param::get("threshold_lidar", threshold_lidar)) {
        cout << "Can not get the value of threshold_lidar" << endl;
        exit(1);
    }
    
}




void LivoxMsgCbk1(const livox_ros_driver::CustomMsgConstPtr& livox_msg_in) {
  livox_data.push_back(livox_msg_in);
  if (livox_data.size() < TO_MERGE_CNT) return;

  pcl::PointCloud<PointType> pcl_in;

  for (size_t j = 0; j < livox_data.size(); j++) {
    auto& livox_msg = livox_data[j];
    auto time_end = livox_msg->points.back().offset_time;
    for (unsigned int i = 0; i < livox_msg->point_num; ++i) {
      PointType pt;
      pt.x = livox_msg->points[i].x;
      pt.y = livox_msg->points[i].y;
      pt.z = livox_msg->points[i].z;
//      if (pt.z < -0.3) continue; // delete some outliers (our Horizon's assembly height is 0.3 meters)
      float s = livox_msg->points[i].offset_time / (float)time_end;
//       ROS_INFO("_s-------- %.6f ",s);
      pt.intensity = livox_msg->points[i].line + s*0.1; // The integer part is line number and the decimal part is timestamp
//      ROS_INFO("intensity-------- %.6f ",pt.intensity);
      pt.curvature = livox_msg->points[i].reflectivity * 0.1;
      // ROS_INFO("pt.curvature-------- %.3f ",pt.curvature);
      pcl_in.push_back(pt);
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
  pcl::toROSMsg(pcl_in, pcl_ros_msg);
  pcl_ros_msg.header.stamp.fromNSec(timebase_ns);
  pcl_ros_msg.header.frame_id = "/livox";
  pub_pcl_out1.publish(pcl_ros_msg);
  livox_data.clear();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "livox_repub");
  ros::NodeHandle nh;

  ROS_INFO("start livox_repub");

  ros::Subscriber sub_livox_msg1 = nh.subscribe<livox_ros_driver::CustomMsg>(
      "/livox/lidar", 100, LivoxMsgCbk1);
  pub_pcl_out1 = nh.advertise<sensor_msgs::PointCloud2>("/livox_pcl0", 100);

  ros::spin();
}
