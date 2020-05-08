/*
 * pcd_to_descriptors.h
 *
 *  Created on: Feb 27, 2013
 *      Author: Kai Franke
 * This program reads a pcd file and publishes its pointcloud
 */

#include "ros/ros.h"
//msg head file
#include <sensor_msgs/PointCloud2.h>
#include <object_recognition/Shot352.h>
#include <object_recognition/Shot352_bundle.h>
#include <object_recognition/Shot1344.h>
#include <object_recognition/Shot1344_bundle.h>
#include <object_recognition/FPFH.h>
#include <object_recognition/FPFH_bundle.h>
#include <geometry_msgs/Vector3.h>

#include <stdio.h>
#include <string>
#include <vector>
//pcl head file
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/search/search.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/common/transforms.h>

#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>

// #include <pcl/conversions.h>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <dynamic_reconfigure/server.h>
#include <object_recognition/cconfigConfig.h>

#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h> //包含fpfh加速计算的omp(多核并行计算)
// #include <pcl/registration/ia_ransac.h>
// #include <pcl/registration/correspondence_estimation.h>
// #include <pcl/registration/correspondence_rejection_features.h> //特征的错误对应关系去除


typedef pcl::PointXYZRGB PointType;//接收点云的格式
typedef pcl::Normal NormalType;//点云法向量格式
typedef pcl::SHOT352 SHOT352;//shot32特征格式
typedef pcl::SHOT1344 SHOT1344;//shot1344特征格式（color）
typedef pcl::FPFHSignature33 FPFH;
typedef pcl::PointCloud<PointType> PointCloud;//接收点云的”存储“格式
typedef pcl::PointCloud<NormalType> NormalCloud;//接收点云法向量存储格式
typedef pcl::PointCloud<SHOT352> DescriptorCloudShot352;//shot32特征存储格式，非输出消息部分
typedef pcl::PointCloud<SHOT1344> DescriptorCloudShot1344;//shot1344特征存储格式（color）
typedef pcl::PointCloud<FPFH> DescriptorCloudFPFH;//特征存储格式

typedef sensor_msgs::PointCloud2 KeypointMsg;//关键点 消息 格式？××
typedef geometry_msgs::Vector3 PointVectorMsg;
typedef object_recognition::Shot352_bundle Shot352Msg;//shot352 消息 格式
typedef object_recognition::Shot1344_bundle Shot1344Msg;//shot1344color 消息 格式
typedef object_recognition::FPFH_bundle FPFHMsg;//FPFH 消息 格式

using namespace std;

// Initialize Publisher for keypoints and descriptors 
ros::Publisher pub_points;//创建 point的发布者
ros::Publisher pub_keypoints;//创建key point的发布者
ros::Publisher pub_descriptors_Shot352;//同 特征发布
ros::Publisher pub_descriptors_Shot1344;//同上
ros::Publisher pub_descriptors_FPFH;//同上

ros::Publisher pub_points_full;
ros::Publisher pub_points_ob;//创建 point的发布者
ros::Publisher pub_keypoints_ob;//创建key point的发布者
ros::Publisher pub_descriptors_Shot352_ob;//同 特征发布
ros::Publisher pub_descriptors_Shot1344_ob;//同上
ros::Publisher pub_descriptors_FPFH_ob;//同上
ros::Publisher pub_points_colora;//同上
ros::Publisher pub_points_vector;//同上

// Point clouds for object, world and its normals, keypoints and descriptors
PointCloud::Ptr cloud_r;//origin 点云指针，cloud开头
PointCloud::Ptr cloud_a;//after pre process 点云指针，cloud开头
PointCloud::Ptr cloud_keypoints;//关键点的
NormalCloud::Ptr cloud_normals;//法向量的
DescriptorCloudShot352::Ptr cloud_descriptors_shot352;//特征的
DescriptorCloudShot1344::Ptr cloud_descriptors_shot1344;
DescriptorCloudFPFH::Ptr cloud_descriptors_FPFH;

KeypointMsg::Ptr output_keypoints;//关键点的，sensor里面的--输出
KeypointMsg::Ptr output_points;//关键点的，sensor里面的--输出
KeypointMsg::Ptr output_colora;//关键点的，sensor里面的--输出
KeypointMsg::Ptr output_points_full;//点的，sensor里面的--输出
Shot352Msg::Ptr output_descriptors_shot352;//特征的，输出部分指针，注意s，指向多个shot？   只是指针用途
Shot1344Msg::Ptr output_descriptors_shot1344;
FPFHMsg::Ptr output_descriptors_FPFH;
PointVectorMsg::Ptr points_vector;

// string pcd_name = "/home/wang/catkin_work/src/object_recognition/obj/";// obj 的 pcd 路径
bool flag_model;
bool flag_world;
bool flag_world_save;
bool apply_cam;
int pcd_name;
int pcd_name_model;

//Algorithm params for Shot descriptor
double cloud_ss, descr_rad ;//特征提取的参数在这里定义
char* output_frame=new char[100];//pcd id name

bool apply_axis;
double axis_filter; 
bool apply_voxel; // set to true if voxel filter should be applied
double voxel_size;// Distance threshold for plane

bool apply_noplane;
double threshold_plane;
bool keep_organized;// if true filtered points will be removed from the point cloud, if false overwritten by NaN

bool apply_octree;
bool reset_octree;
bool reset_octree_once;
double resolution;// int max_window_size;
int noise_filter;
// double initial_distance;
// double max_dist;

bool apply_outlier;
double outlier_Thresh;
int outlier_meanK;

bool apply_segrgb;
float dist_thre;
float point_color_thre;
float region_color_thre;
int min_cluster;
int seg_num;
int current_num;

bool spincall;
bool ShutDown;

bool apply_shot352;
bool apply_shot1344;
bool apply_fpfh;

// Declare pointcloud object, for calculating pointclouds and texture mappings
rs2::pointcloud* pc_pointer;
// We want the points object to be persistent so we can display the last cloud when a frame drops
rs2::points* points_pointer;
rs2::pipeline* pipe_point;

pcl::NormalEstimationOMP<PointType, NormalType> norm_est_cloud;//法向量估计的类实例化
pcl::PointCloud<int> sampled_indices_cloud;//采样后的 指数 点云 index
pcl::UniformSampling<PointType> uniform_sampling_cloud;//降采样class
pcl::SHOTEstimationOMP<PointType, NormalType, SHOT352> descr_est_shot352;//特征提取类实例
pcl::SHOTColorEstimationOMP<PointType, NormalType, SHOT1344> descr_est_shot1344;//shotcolor提取实例
pcl::FPFHEstimationOMP<PointType,NormalType,FPFH> descr_est_fpfh;

//set up the passthrongh filter for object save
pcl::PassThrough<PointType> pt(false);//false means take the inlier instead of the outlier

//use for the grid /downsample
pcl::VoxelGrid<PointType> sor_grid;

//use for the outlier 
pcl::StatisticalOutlierRemoval<PointType> sor_outlier;

// Declare the segmentation object for planes
pcl::SACSegmentation<PointType> seg_plane;

// Declare the filtering object for planes
pcl::ExtractIndices<PointType> extract_planes;

void Shot352toROSMsg(const DescriptorCloudShot352 &input, Shot352Msg &output);
void Shot1344toROSMsg(const DescriptorCloudShot1344 &input, Shot1344Msg &output);
void FPFHtoROSMsg(const DescriptorCloudFPFH &input, FPFHMsg &output);

PointCloud::Ptr PCL_Conversion(const rs2::points& points, const rs2::video_frame& color);
void paramCallback(object_recognition::cconfigConfig &config, uint32_t level);
double computeCloudResolution (const pcl::PointCloud<PointType>::ConstPtr &cloud);
