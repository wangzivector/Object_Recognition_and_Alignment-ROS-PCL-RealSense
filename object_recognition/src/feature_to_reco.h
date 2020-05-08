
#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3.h>
#include <stdio.h>
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <tf/transform_broadcaster.h>
#include <object_recognition/Shot352_bundle.h>
#include <object_recognition/Shot1344_bundle.h>
#include <object_recognition/FPFH_bundle.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <dynamic_reconfigure/server.h>
#include <object_recognition/rconfigConfig.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/histogram_visualizer.h> //直方图的可视化
#include <pcl/visualization/pcl_plotter.h>
#include <boost/thread/thread.hpp>

#include <Eigen/Core>
#include <pcl/registration/icp.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_features.h> //特征的错误对应关系去除
#include <pcl/registration/correspondence_rejection_sample_consensus.h> //随机采样一致性去除
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/region_growing_rgb.h>

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef pcl::SHOT352 SHOT352;
typedef pcl::SHOT1344 SHOT1344;
typedef pcl::FPFHSignature33 FPFH;
typedef pcl::PointCloud<SHOT352> DescriptorCloudShot352;
typedef pcl::PointCloud<SHOT1344> DescriptorCloudShot1344;
typedef pcl::PointCloud<FPFH> DescriptorCloudFPFH;
typedef object_recognition::Shot352_bundle Shot352Msg;
typedef object_recognition::Shot1344_bundle Shot1344Msg;
typedef object_recognition::FPFH_bundle FPFHMsg;
typedef sensor_msgs::PointCloud2 PointCloudROS;
typedef geometry_msgs::Vector3 PointVectorMsg;
using namespace std;

// Initialize Subscriber
ros::Subscriber sub_keypoint_object;
ros::Subscriber sub_keypoint_world;
ros::Subscriber sub_point_world;
ros::Subscriber sub_point_object;
ros::Subscriber sub_descriptors_object_shot352;
ros::Subscriber sub_descriptors_world_shot352;
ros::Subscriber sub_descriptors_object_shot1344;
ros::Subscriber sub_descriptors_object_FPFH;
ros::Subscriber sub_descriptors_world_shot1344;
ros::Subscriber sub_descriptors_world_FPFH;
ros::Subscriber sub_points_vector;

// Publisher for debug output
ros::Publisher pub_object_corr_clus;
ros::Publisher pub_world_corr_clus;
ros::Publisher pub_object_corr;
ros::Publisher pub_world_corr;
ros::Publisher pub_object_align;
ros::Publisher pub_object_icp;
ros::Publisher pub_object_inverse;
ros::Publisher pub_world_color;

//Algorithm params
double cg_size_, cg_thresh_, max_descr_dist_;
double max_correspondence_distance;
double min_sample_distance;
int number_samples;
int randomness;
int max_iterations;
double similar_thre;

bool ShutDown;
bool apply_sacia;
bool apply_icp;
double max_corr_distance;
int max_iter_icp;
double transformation;
double euclidean_Fitness;


bool apply_FpfhsaciaIcp;
bool apply_NDT;
float ndt_transepsilon;
float ndt_stepsize;
float ndt_resolution;
int ndt_maxiteration;

int seg_num;
int current_num;
int seg_points;
double high_score;
bool re_count;

bool apply_RANSAC;

std::vector <PointCloud> seg_cloud_vector;
std::vector <DescriptorCloudFPFH> seg_fpfh_vector;

// Point clouds for object, world and its normals, keypoints and descriptors
PointCloud::Ptr object_keypoints;
PointCloud::Ptr world_keypoints;
PointCloud::Ptr world_points;
PointCloud::Ptr object_points;
DescriptorCloudShot352::Ptr object_descriptors_shot352;
DescriptorCloudShot352::Ptr world_descriptors_shot352;
DescriptorCloudShot1344::Ptr object_descriptors_shot1344;
DescriptorCloudFPFH::Ptr object_descriptors_FPFH;
DescriptorCloudShot1344::Ptr world_descriptors_shot1344;
DescriptorCloudFPFH::Ptr world_descriptors_FPFH;
PointCloud::Ptr aligned_pointcloud;
PointCloudROS::Ptr aligned_rospc;
PointCloud::Ptr icp_pointcloud;
PointCloudROS::Ptr icp_rospc;
PointCloud::Ptr inverse_pointcloud;
PointCloudROS::Ptr inverse_rospc;
PointCloud::Ptr colored_cloud;
PointCloudROS::Ptr colored_rospc;
PointVectorMsg::Ptr points_vector;
PointCloud::Ptr seg_cloud;
DescriptorCloudFPFH::Ptr seg_fpfh;


void shot352fromROSMsg (const object_recognition::Shot352_bundle &input,  DescriptorCloudShot352 &output);
void shot1344fromROSMsg (const object_recognition::Shot1344_bundle &input,  DescriptorCloudShot1344 &output);
void FPFHfromROSMsg (const object_recognition::FPFH_bundle &input,  DescriptorCloudFPFH &output);


//topic callback function 
void world_point_cb (const sensor_msgs::PointCloud2ConstPtr& input);
void world_keypoint_cb (const sensor_msgs::PointCloud2ConstPtr& input);
void object_point_cb (const sensor_msgs::PointCloud2ConstPtr& input);
void object_keypoint_cb (const sensor_msgs::PointCloud2ConstPtr& input);
void world_descriptor_shot352_cb (const object_recognition::Shot352_bundle::Ptr input);
void world_descriptor_shot1344_cb (const object_recognition::Shot1344_bundle::Ptr input);
void world_descriptor_FPFH_cb (const object_recognition::FPFH_bundle::Ptr input);
void object_descriptor_shot352_cb (const object_recognition::Shot352_bundle::Ptr input);
void object_descriptor_shot1344_cb (const object_recognition::Shot1344_bundle::Ptr input);
void object_descriptor_FPFH_cb (const object_recognition::FPFH_bundle::Ptr input);
void points_vector_cb (const geometry_msgs::Vector3ConstPtr& input);

pcl::SampleConsensusInitialAlignment<PointType, PointType, FPFH> SACIA;
pcl::IterativeClosestPoint<PointType, PointType> ICP;
pcl::SampleConsensusPrerejective<PointType,PointType,FPFH> SACPJ;


void FpfhsaciaIcp ();
void NDTRegistration ();
void RANSACRegistration ();
void cluster (const pcl::CorrespondencesPtr &object_world_corrs);
void paramCallback (object_recognition::rconfigConfig &config, uint32_t level);