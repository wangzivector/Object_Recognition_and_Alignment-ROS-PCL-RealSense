
#include "feature_to_reco.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "feature_to_reco");
	ros::NodeHandle nh;
	ros::NodeHandle nh_param("~");
    dynamic_reconfigure::Server<object_recognition::rconfigConfig> server;
    dynamic_reconfigure::Server<object_recognition::rconfigConfig>::CallbackType f;
    f = boost::bind(&paramCallback, _1, _2);
    server.setCallback(f);

	world_points = PointCloud::Ptr (new PointCloud());
	object_points = PointCloud::Ptr (new PointCloud());
	world_keypoints = PointCloud::Ptr (new PointCloud());
	object_keypoints = PointCloud::Ptr (new PointCloud());
	world_descriptors_shot352 = DescriptorCloudShot352::Ptr (new DescriptorCloudShot352 ());
	world_descriptors_shot1344 = DescriptorCloudShot1344::Ptr (new DescriptorCloudShot1344 ());
	world_descriptors_FPFH = DescriptorCloudFPFH::Ptr (new DescriptorCloudFPFH ());
	object_descriptors_shot352 = DescriptorCloudShot352::Ptr (new DescriptorCloudShot352 ());
	object_descriptors_shot1344 = DescriptorCloudShot1344::Ptr (new DescriptorCloudShot1344 ());
	object_descriptors_FPFH = DescriptorCloudFPFH::Ptr (new DescriptorCloudFPFH ());
	aligned_pointcloud = PointCloud::Ptr (new PointCloud);
	aligned_rospc = PointCloudROS::Ptr (new PointCloudROS);
	icp_pointcloud = PointCloud::Ptr (new PointCloud);
	icp_rospc = PointCloudROS::Ptr (new PointCloudROS);
	inverse_pointcloud = PointCloud::Ptr (new PointCloud);
	inverse_rospc = PointCloudROS::Ptr (new PointCloudROS);
	colored_cloud = PointCloud::Ptr (new PointCloud ());
	colored_rospc = PointCloudROS::Ptr (new PointCloudROS);
	points_vector = PointVectorMsg::Ptr (new PointVectorMsg);
	seg_cloud = PointCloud::Ptr (new PointCloud());
	seg_fpfh = DescriptorCloudFPFH::Ptr (new DescriptorCloudFPFH ());
	

		// Create a ROS subscriber for the object and world keypoints and descriptors
	sub_point_world  = nh.subscribe ("/kinect_pointcloud/keypoints/points",  1, world_point_cb );
	sub_keypoint_world  = nh.subscribe ("/kinect_pointcloud/keypoints/keypoints",  1, world_keypoint_cb );
	sub_point_object  = nh.subscribe ("/kinect_pointcloud/keypoints/points_ob",  1, object_point_cb);
	sub_keypoint_object = nh.subscribe ("/kinect_pointcloud/keypoints/keypoints_ob", 1, object_keypoint_cb);
	sub_descriptors_world_shot352  = nh.subscribe ("/kinect_pointcloud/descriptors/Shot352"  , 1, world_descriptor_shot352_cb );
	sub_descriptors_object_shot352 = nh.subscribe ("/kinect_pointcloud/descriptors/Shot352_ob" , 1, object_descriptor_shot352_cb);
	sub_descriptors_world_shot1344 = nh.subscribe ("/kinect_pointcloud/descriptors/Shot1344" , 1, world_descriptor_shot1344_cb );	
	sub_descriptors_object_shot1344= nh.subscribe ("/kinect_pointcloud/descriptors/Shot1344_ob", 1, object_descriptor_shot1344_cb);
	sub_descriptors_world_FPFH = nh.subscribe ("/kinect_pointcloud/descriptors/FPFH" , 1, world_descriptor_FPFH_cb );
	sub_descriptors_object_FPFH= nh.subscribe ("/kinect_pointcloud/descriptors/FPFH_ob", 1, object_descriptor_FPFH_cb);
	sub_points_vector = nh.subscribe ("/kinect_pointcloud/points_vector", 1, points_vector_cb);
	
	// pub_object_corr_clus = nh.advertise<PointCloudROS> ("correspondences/object/clustered", 1);	
	// pub_world_corr_clus = nh.advertise<PointCloudROS> ("correspondences/world/clustered", 1);
	// pub_object_corr = nh.advertise<PointCloudROS> ("correspondences/object/corred", 1);	
	// pub_world_corr = nh.advertise<PointCloudROS> ("correspondences/world/corred", 1);

	pub_object_align = nh.advertise<PointCloudROS> ("/kinect_pointcloud/object/aligned", 1);	
	pub_object_icp = nh.advertise<PointCloudROS> ("/kinect_pointcloud/object/icp", 1);	
	pub_object_inverse = nh.advertise<PointCloudROS> ("/kinect_pointcloud/object/inverse", 1);	
	pub_world_color = nh.advertise<PointCloudROS> ("/kinect_pointcloud/keypoints/points_color", 1);	

	ROS_INFO("ros spin start ...");
	ros::spin();
    return 0;
}


//===================================================
//  point_vector_cb
//  function for vectors used to call the segmentaion 
//  num of one world   this fun set the seg_num and 
//  current_num , make a profile of single world reco
//=================================================== 
void points_vector_cb (const geometry_msgs::Vector3ConstPtr& input)
{
	if(((int)object_descriptors_FPFH->size() == 0) || ((int)world_descriptors_FPFH->size() == 0)) 
	{
		ROS_WARN("please send object first !!! Warn Received %i FPFH object and %i FPFH world. ", (int)object_descriptors_FPFH->size(), (int)world_descriptors_FPFH->size());
		return;
	}
	// check if the received object descriptors can be assigned to the stored keypoints
	if ((int)object_keypoints->size() != (int)object_descriptors_FPFH->size())
	{
		ROS_WARN("Received %i descriptors and %i keypoints for the object. Number must be equal", (int)object_descriptors_FPFH->size(), (int)object_keypoints->size());
		return;
	}
	// check if the stored world descriptors can be assinged to the stored keypoints
	if ((int)world_keypoints->size() != (int)world_descriptors_FPFH->size())
	{
		ROS_WARN("Received %i descriptors and %i keypoints for the world. Number must be equal", (int)world_descriptors_FPFH->size(), (int)world_keypoints->size());
		return;
	}
		ROS_INFO("successfully store %i descriptors for world ( %i descriptors for object)", (int)world_descriptors_FPFH->size(), (int)object_descriptors_FPFH->size());

	seg_num = input->x;
	current_num = input->y;
	seg_points = input->z;
	ROS_INFO("seg_num:%d  current_num:%d  push_back points and fpfh %d to vector... ", seg_num, current_num, seg_points);
	

	if ((seg_num == current_num) && (current_num != 0)) 
	{
		high_score == 0;
		seg_cloud_vector.clear ();
		seg_fpfh_vector.clear ();
		re_count = false;
	}

	if (re_count)
	{
		ROS_WARN ("vector request aborted recount needed ");
		return;
	} 

	if ((seg_num - current_num) != (int)seg_cloud_vector.size ())
	{
		ROS_WARN ("WARN !!! count by  seg_num %d  current_num %d  seg_vector %d", seg_num, current_num,(int) seg_cloud_vector.size ());
		seg_cloud_vector.clear ();
		seg_fpfh_vector.clear ();
		re_count = true;
		return;
	}else re_count = false;


	if ((seg_points == (int)world_keypoints->size ()) && (seg_points == (int)world_descriptors_FPFH->size ()))
	{	
		seg_cloud_vector.push_back (*world_keypoints);
		seg_fpfh_vector.push_back (*world_descriptors_FPFH);
	}else {
		ROS_WARN ("wrong receive point num :  %d world_keypoints / %d world_descriptors_FPFH    but %d seg_points", \
			(int)world_keypoints->size (), (int)world_descriptors_FPFH->size (), seg_points);
		re_count = true;
	}

	if ((current_num == 1))
	{
		if (((int)seg_cloud_vector.size () == seg_num))
		{
			seg_num = 0;
			current_num = 0;
			ROS_WARN ("start recognizer () .. ");
			// ros::Duration (1).sleep ();
			// start recognizer 
			if (apply_FpfhsaciaIcp) FpfhsaciaIcp ();
			if (apply_NDT) NDTRegistration (); // target does't need grid ,means it can use points cloud 
			if (apply_RANSAC) RANSACRegistration();
		}else{
			ROS_WARN ("wrong size:  %d seg_num  / %d seg_cloud_vector ", seg_num, (int)seg_cloud_vector.size ());
			re_count = true;
		}
	}
}


//===================================================
//  RANSACRegistration
//  SampleConsensusPrerejective to get position 
//  recognition to get the matrix transform the model
//=================================================== 
void RANSACRegistration ()
{
		ros::Time Begin = ros::Time::now();//use for time measure 
	high_score = 1;
	for(int Num = 0; Num < seg_cloud_vector.size(); Num ++)
	{
		ROS_WARN ("fpfh_recognizer make process %d/%d of %d points", Num,  (int)seg_cloud_vector.size(),  (int)seg_cloud_vector[Num].size ());
		pcl::copyPointCloud (seg_cloud_vector[Num], *seg_cloud);
		pcl::copyPointCloud (seg_fpfh_vector[Num], *seg_fpfh);
		pcl::console::print_info ("object_keypoints/FPFH: %d/%d    seg_cloud/seg_fpfh: %d/%d \n", object_keypoints->size(), object_descriptors_FPFH->size(), seg_cloud->size(), seg_fpfh->size());
		// Perform alignment
		pcl::console::print_highlight ("SACPJ Starting alignment...\n");
		SACPJ.setInputSource (object_keypoints);
		SACPJ.setSourceFeatures (object_descriptors_FPFH);
		SACPJ.setInputTarget (seg_cloud);
		SACPJ.setTargetFeatures (seg_fpfh);
		SACPJ.setMaximumIterations (max_iterations); // Number of RANSAC iterations
		SACPJ.setNumberOfSamples (number_samples); // Number of points to sample for generating/prerejecting a pose
		SACPJ.setCorrespondenceRandomness (randomness); // Number of nearest features to use
		SACPJ.setSimilarityThreshold (similar_thre); // Polygonal edge length similarity threshold
		SACPJ.setMaxCorrespondenceDistance (max_corr_distance); // Inlier threshold
		SACPJ.setInlierFraction (min_sample_distance); // Required inlier fraction for accepting a pose hypothesis
		SACPJ.align (*aligned_pointcloud);
		
		if (SACPJ.hasConverged ())
		{
			// Print results
			printf ("\n");
			Eigen::Matrix4f transformation = SACPJ.getFinalTransformation ();
			pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
			pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
			pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
			pcl::console::print_info ("\n");
			pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
			pcl::console::print_info ("\n");
			pcl::console::print_info ("Inliers: %i/%i\n", SACPJ.getInliers ().size (), object_keypoints->size ());
		}
		if (high_score < SACPJ.getInliers ().size ())
		{
			high_score = SACPJ.getInliers ().size ();
			pcl::toROSMsg (*aligned_pointcloud, *aligned_rospc);
		}
		pcl::toROSMsg (*seg_cloud, *colored_rospc);
		colored_rospc->header.frame_id = "map";//map id for rviz i think 
		pub_world_color.publish(colored_rospc);
	}
	pub_object_align.publish(aligned_rospc);
		ros::Time Stop = ros::Time::now();//use for time measure 
		ROS_INFO(" sacpj time : %.2f s   sacpj score is : %f", (Stop.toSec () - Begin.toSec ()), high_score);	

}

//===================================================
//  NDTRegistration
//  Normal Distributions Transform to get position 
//  recognition to get the matrix transform the model
//=================================================== 
void NDTRegistration ()
{
		ros::Time Begin = ros::Time::now();//use for time measure 
	cout << "start NDTRegistration ..." << endl;
	high_score = 1;
	for(int Num = 0; Num < seg_cloud_vector.size(); Num ++)
	{
		ROS_WARN ("NDTR %d points", (int)object_keypoints->size());
		ROS_WARN ("NDTRegistration make process %d/%d of %d points", Num,  (int)seg_cloud_vector.size(),  (int)seg_cloud_vector[Num].size ());
		pcl::copyPointCloud (seg_cloud_vector[Num], *seg_cloud);
		
		// Initializing Normal Distributions Transform (NDT).
		pcl::NormalDistributionsTransform<PointType, PointType> ndt;
		// Setting scale dependent NDT parameters
		ndt.setTransformationEpsilon (ndt_transepsilon);  // Setting minimum transformation difference for termination condition.
		ndt.setStepSize (ndt_stepsize); // Setting maximum step size for More-Thuente line search.
		ndt.setResolution (ndt_resolution); //Setting Resolution of NDT grid structure (VoxelGridCovariance).// must be adjusted before use 
		ndt.setMaximumIterations (ndt_maxiteration);  // Setting max number of registration iterations.
		ndt.setInputSource (seg_cloud);  // Setting point cloud to be aligned.
		ndt.setInputTarget (object_keypoints);  // Setting point cloud to be aligned to.

		// Set initial alignment estimate found using robot odometry.
		Eigen::AngleAxisf init_rotation (0.0, Eigen::Vector3f::UnitZ ());
		Eigen::Translation3f init_translation (0.05, -0.04, -0.2);
		Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();

		// Calculating required rigid transform to align the input cloud to the target cloud.
		ndt.align (*aligned_pointcloud, init_guess);

		float NDT_score = ndt.getFitnessScore ();
		std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()
					<< " score: " << NDT_score << endl << "matrix ; " << ndt.getFinalTransformation () << std::endl;
		if (NDT_score < high_score)
		{
			high_score = NDT_score;
			pcl::toROSMsg(*aligned_pointcloud, *aligned_rospc);
		}
		pcl::toROSMsg (*seg_cloud, *colored_rospc);
		colored_rospc->header.frame_id = "map";//map id for rviz i think 
		pub_world_color.publish(colored_rospc);
	}
	pub_object_align.publish(aligned_rospc);

		ros::Time Stop = ros::Time::now();//use for time measure 
		ROS_INFO("done ndt time : %.2f s   icp score is : %f", (Stop.toSec () - Begin.toSec ()), high_score);	
}

//===================================================
//  FpfhsaciaIcp
//  SACIA and ICP method to get position recognition
//  to get the matrix transform the model into world
//=================================================== 
void FpfhsaciaIcp()
{
	high_score = 1;
	for(int Num = 0; Num < seg_cloud_vector.size(); Num ++)
	{
		ROS_WARN ("fpfh_recognizer make process %d/%d of %d points", Num,  (int)seg_cloud_vector.size(),  (int)seg_cloud_vector[Num].size ());
		pcl::copyPointCloud (seg_cloud_vector[Num], *seg_cloud);
		pcl::copyPointCloud (seg_fpfh_vector[Num], *seg_fpfh);
		
		//
		//   sacia for a rough align
		//
		if(!apply_sacia) return; 
			ros::Time Begin = ros::Time::now();//use for time measure
		// sac alien start ..
		pcl::SampleConsensusInitialAlignment<PointType, PointType, FPFH> SACIA;
		SACIA.setInputSource (seg_cloud);
		SACIA.setSourceFeatures (seg_fpfh);
		SACIA.setInputTarget (object_keypoints);
		SACIA.setTargetFeatures (object_descriptors_FPFH);

		SACIA.setNumberOfSamples (number_samples);  //设置每次迭代计算中使用的样本数量（可省）,可节省时间
		SACIA.setCorrespondenceRandomness (randomness); //设置计算协方差时选择多少近邻点，该值越大，协防差越精确，但是计算效率越低.(可省)
		SACIA.setMinSampleDistance (min_sample_distance);// we’ve decided to truncate the error with an upper limit of 0.01 squared.
		SACIA.setMaxCorrespondenceDistance (max_correspondence_distance);//the maximum correspondence distance is actually specified as squared distance; 
		SACIA.setMaximumIterations (max_iterations);
		SACIA.align(*aligned_pointcloud); 
		float sacia_fitness_score = (float) SACIA.getFitnessScore (max_correspondence_distance);
		Eigen::Matrix4f matrix1 = SACIA.getFinalTransformation ();

		ros::Time Stop = ros::Time::now(); 
		ROS_INFO(" sacia time : %.2f s    sacia score is :%f   ", (Stop.toSec() - Begin.toSec()), sacia_fitness_score);

		// printf ("    | %6.3f %6.3f %6.3f | \n", matrix1 (0,0), matrix1 (0,1), matrix1 (0,2));
		// printf ("R = | %6.3f %6.3f %6.3f | \n", matrix1 (1,0), matrix1 (1,1), matrix1 (1,2));
		// printf ("    | %6.3f %6.3f %6.3f | \n", matrix1 (2,0), matrix1 (2,1), matrix1 (2,2));
		// cout << endl <<endl;


		if(!apply_icp) return;
			Begin = ros::Time::now();
		//
		//   icp for percise align	
		//
		ICP.setInputSource(aligned_pointcloud);//how abaot icpwithnolinear and gerneral icp
		ICP.setInputTarget(object_keypoints);
		ICP.setMaxCorrespondenceDistance(max_corr_distance);//Set the max correspondence distance to 4cm (e.g., correspondences with higher distances will be ignored)
		ICP.setMaximumIterations(max_iter_icp);// 最大迭代次数
		ICP.setTransformationEpsilon(transformation);	// 两次变化矩阵之间的差值//setTransformationEpsilon， 前一个变换矩阵和当前变换矩阵的差异小于阈值时，就认为已经收敛了，是一条收敛条件
		ICP.setEuclideanFitnessEpsilon(euclidean_Fitness);	//收敛条件是均方误差和小于阈值， 停止迭代。
		ICP.align(*icp_pointcloud);
		Eigen::Matrix4f matrix2 = ICP.getFinalTransformation();
		float icp_fitness_score = ICP.getFitnessScore ();
			Stop = ros::Time::now(); 
		ROS_INFO("done ICP time : %.2f s   icp score is : %f", (Stop.toSec () - Begin.toSec ()), icp_fitness_score);
		// printf ("    | %6.3f %6.3f %6.3f | \n", matrix2 (0,0), matrix2 (0,1), matrix2 (0,2));
		// printf ("R = | %6.3f %6.3f %6.3f |  of icp matrics \n", matrix2 (1,0), matrix2 (1,1), matrix2 (1,2));
		// printf ("    | %6.3f %6.3f %6.3f | \n", matrix2 (2,0), matrix2 (2,1), matrix2 (2,2));

		// Eigen::Matrix3f rotation1 = matrix1.block<3,3>(0, 0);
		// Eigen::Vector3f translation1 = matrix1.block<3,1>(0, 3);
		Eigen::Matrix4f matrix3;
		matrix3 = matrix2 * matrix1;
		// cout <<"\n matrix3 : \n" << matrix3.inverse() << endl;
		//矩阵求逆
		pcl::transformPointCloud(*object_keypoints, *inverse_pointcloud, matrix3.inverse());
		cout << endl << " **icp processed  score: " << icp_fitness_score << endl;
		if (high_score > icp_fitness_score){
			high_score = icp_fitness_score;
			pcl::toROSMsg(*aligned_pointcloud, *aligned_rospc);
			pcl::toROSMsg(*inverse_pointcloud, *inverse_rospc);
			pcl::toROSMsg(*icp_pointcloud, *icp_rospc);
		}
		pcl::toROSMsg (*seg_cloud, *colored_rospc);
		colored_rospc->header.frame_id = "map";//map id for rviz i think 
		pub_world_color.publish(colored_rospc);
	}
	ROS_WARN ("all in icp final score is %f", high_score);
	pub_object_align.publish(aligned_rospc);
	pub_object_inverse.publish(inverse_rospc);
	pub_object_icp.publish(icp_rospc);
}	

//===================================================
//  paramcallback 
//  use for dynamic param service 
//=================================================== 
void paramCallback(object_recognition::rconfigConfig &config, uint32_t level) {
	apply_sacia = config.apply_sacia;
	apply_icp = config.apply_icp;
	max_iterations = config.max_iterations;//500
	randomness = config.randomness;//设置计算协方差时选择多少近邻点，该值越大，协防差越精确，但是计算效率越低.(可省)
	number_samples = config.number_samples;//20
	max_correspondence_distance = config.max_corr_distance*config.max_corr_distance;//
	ShutDown = config.CLOSE;
	min_sample_distance = config.min_sample_distance;
	euclidean_Fitness = config.euclidean_Fitness;
	transformation = config.transformation * 1.0e-10;
	max_iter_icp = config.max_iter_icp;	
	max_corr_distance = config.max_corr_dist_icp;
	ndt_transepsilon = config.ndt_transepsilon;
	ndt_stepsize = config.ndt_stepsize;
	ndt_resolution = config.ndt_resolution;
	ndt_maxiteration = config.ndt_maxiteration;
	apply_RANSAC = config.apply_RANSAC;

	apply_FpfhsaciaIcp = config.apply_FpfhsaciaIcp;
	apply_NDT = config.apply_NDT;

	ROS_WARN(" Reconfigure server got!");
	if(ShutDown) ros::shutdown();

}



//===================================================
//  world_keypoint_cb
// Callback function when world keypoints are received
//=================================================== 
void world_keypoint_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	pcl::fromROSMsg(*input, *world_keypoints);
	ROS_INFO("world_keypoints size: %d points", (int)world_keypoints->size());
	// pcl::toROSMsg(*world_keypoints, *icp_rospc);
	// pub_object_icp.publish(icp_rospc);
}


//===================================================
//  object_keypoint_cb
// Callback function when object keypoints are received
//=================================================== 
void object_keypoint_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	pcl::fromROSMsg(*input, *object_keypoints);
	ROS_INFO("object_keypoints size: %d points", (int)object_keypoints->size());
}

//===================================================
//  world_point_cb
// Callback function when world keypoints are received
//=================================================== 
void world_point_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	pcl::fromROSMsg(*input, *world_points);
	ROS_INFO("world_points size: %d points", (int)world_points->size());
}

//===================================================
//  object_point_cb
// Callback function when world keypoints are received
//=================================================== 
void object_point_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	pcl::fromROSMsg(*input, *object_points);
	ROS_INFO("object_points size: %d points", (int)object_points->size());
}

//===================================================
//  object_descriptor_shot352_cb
// callback function when object descriptors are received
//=================================================== 
void object_descriptor_shot352_cb (const object_recognition::Shot352_bundle::Ptr input)
{
	shot352fromROSMsg(*input, *object_descriptors_shot352);
	ROS_INFO("object_descriptors_shot352 size: %d points", (int)object_descriptors_shot352->size());
}


//===================================================
//  world_descriptor_shot352_cb
// this will trigger the recognition if all the other 
// keypoints and descriptors have been received
// Callback function when the world descriptors are received
//=================================================== 
void world_descriptor_shot352_cb (const object_recognition::Shot352_bundle::Ptr input)
{
	shot352fromROSMsg(*input, *world_descriptors_shot352);
	ROS_INFO("world_descriptors_shot352 size: %d points", (int)world_descriptors_shot352->size());

	if(((int)world_descriptors_shot352->size() == 0) || ((int)object_descriptors_shot352->size() == 0)) 
	{
		ROS_WARN("Warn Received %i shot352 object and %i shot352 world. ", (int)object_descriptors_shot352->size(), (int)world_descriptors_shot352->size());
		return;
	}

	// check if the stored world descriptors can be assinged to the stored keypoints
	if ((int)world_keypoints->size() != (int)world_descriptors_shot352->size())
	{
		ROS_WARN("Received %i shot352 and %i keypoints for the world. Number must be equal", (int)world_descriptors_shot352->size(), (int)world_keypoints->size());
		return;
	}
	// check if the received object descriptors can be assigned to the stored keypoints
	if ((int)object_keypoints->size() != (int)object_descriptors_shot352->size())
	{
		ROS_WARN("Received %i shot352 and %i keypoints for the object. Number must be equal", (int)input->descriptors.size(), (int)object_keypoints->size());
		return;
	}
	
	ROS_INFO(" %i shot352 for the world // %i shot352 for the object \n start to recognition process", (int)world_descriptors_shot352->size(), (int)input->descriptors.size());


    // pcl::visualization::PCLHistogramVisualizer view;
	// view.setBackgroundColor(0,0,100);
	// view.addFeatureHistogram<SHOT352> (*object_descriptors_shot352,"SHOT352",10);   //对下标为1000的元素可视化
	// view.spinOnce(100);  //循环的次数

	// pcl::visualization::PCLPlotter plotter;
	// // We need to set the size of the descriptor beforehand.
	// plotter.addFeatureHistogram(object_descriptors_shot352, 300); //设置的很坐标长度，该值越大，则显示的越细致
	// plotter.plot();

	//
	//  Find Object-World Correspondences with KdTree
	//
	cout << "... finding correspondences ..." << endl;
	pcl::CorrespondencesPtr object_world_corrs (new pcl::Correspondences ());
	

	pcl::KdTreeFLANN<SHOT352> match_search;
	match_search.setInputCloud (object_descriptors_shot352);

	// For each world keypoint descriptor
	// find nearest neighbor into the object keypoints descriptor cloud 
	// and add it to the correspondences vector
	int nan_num = 0, far_num = 0, got_num = 0; 
	for (size_t i = 0; i < world_descriptors_shot352->size (); ++i)
	{
		std::vector<int> neigh_indices (1);
		std::vector<float> neigh_sqr_dists (1);
		if (!pcl_isfinite (world_descriptors_shot352->at (i).descriptor[0])) //skipping NaNs
		{
			ROS_WARN("skin point... index: %d", (int) i);
			nan_num ++;
			continue;
		}
		
		int found_neighs = match_search.nearestKSearch (world_descriptors_shot352->at (i), 1, neigh_indices, neigh_sqr_dists);
		// add match only if the squared descriptor distance is less than 0.25 
		// SHOT descriptor distances are between 0 and 1 by design
		max_descr_dist_ = 0.25;// need to be adjust
		if(found_neighs == 1 && neigh_sqr_dists[0] < (float)max_descr_dist_) 
		{
			pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]); // save as objectpoint' indices and distance  
			object_world_corrs->push_back (corr);
			// cout << "nearest distance is : " << neigh_sqr_dists[0]<< "  got one distance is : " << i << endl;
			got_num ++;
		}
		else 
		{
			// cout << "far distance is : " << neigh_sqr_dists[0]<<endl;
			far_num ++;
		}
	}
	ROS_INFO("frame info : skin : %d , far : %d , got : %d ", nan_num, far_num, got_num);
	std::cout << "from world's point there Correspondences to object found: " << object_world_corrs->size () << std::endl;
	string pcd_path = "/home/wang/catkin_work/src/object_recognition/obj/shot352.pcd";
	pcl::io::savePCDFile<SHOT352>(pcd_path, *world_descriptors_shot352);
	cout<<"saved shot352...\n";
	//
	// all keypoints and descriptors were found, no match the correspondences to the real object!
	//
	cluster(object_world_corrs);
}



//===================================================
//  world_descriptor_shot1344_cb
// Callback function when world descriptors shot1344
//=================================================== 
void world_descriptor_shot1344_cb (const object_recognition::Shot1344_bundle::Ptr input)
{
	shot1344fromROSMsg(*input, *world_descriptors_shot1344);
	ROS_INFO("world_descriptors_shot1344 size: %d points", (int)world_descriptors_shot1344->size());
}

//===================================================
//  object_descriptor_shot1344_cb
// callback function when the object descriptors are received
// this will also trigger the recognition if all t
// he other keypoints and descriptors have been received
//=================================================== 
void object_descriptor_shot1344_cb (const object_recognition::Shot1344_bundle::Ptr input)
{
	shot1344fromROSMsg(*input, *object_descriptors_shot1344);
	ROS_INFO("object shot1344 Received %d descriptors ",(int)object_descriptors_shot352->size());
	// ros::NodeHandle nh_param("~");
	// nh_param.param<double>("maximum_descriptor_distance" , max_descr_dist_ , 0.25 );
	// // check if world was already processed
	// if (world_descriptors_shot1344 == NULL)
	// {
	// 	ROS_WARN("Received object descriptors before having a world pointcloud to compare");
	// 	return;
	// }
	// // check if the stored world descriptors can be assinged to the stored keypoints
	// if ((int)world_keypoints->size() != (int)world_descriptors_shot1344->size())
	// {
	// 	ROS_WARN("Received %i descriptors and %i keypoints for the world. Number must be equal", (int)world_descriptors_shot1344->size(), (int)world_keypoints->size());
	// 	return;
	// }
	// // check if the received object descriptors can be assigned to the stored keypoints
	// if ((int)object_keypoints->size() != (int)input->descriptors.size())
	// {
	// 	ROS_WARN("Received %i descriptors and %i keypoints for the object. Number must be equal", (int)input->descriptors.size(), (int)object_keypoints->size());
	// 	return;
	// }
	

	// Debug output 
	// ROS_INFO("Received %i descriptors for the world and %i for the object", (int)world_descriptors_shot1344->size(), (int)input->descriptors.size());

	// //
	// //  Find Object-World Correspondences with KdTree
	// //
	// cout << "... finding correspondences ..." << endl;
	// pcl::CorrespondencesPtr object_world_corrs (new pcl::Correspondences ());
	
	// pcl::KdTreeFLANN<SHOT1344> match_search;
	// match_search.setInputCloud (object_descriptors_shot1344);
		
	// // For each world keypoint descriptor
	// // find nearest neighbor into the object keypoints descriptor cloud 
	// // and add it to the correspondences vector
	// for (size_t i = 0; i < world_descriptors_shot1344->size (); ++i)
	// {
	// 	std::vector<int> neigh_indices (1);
	// 	std::vector<float> neigh_sqr_dists (1);
	// 	if (!pcl_isfinite (world_descriptors_shot1344->at (i).descriptor[0])) //skipping NaNs
	// 	{
	// 		continue;
	// 	}
	// 	int found_neighs = match_search.nearestKSearch (world_descriptors_shot1344->at (i), 1, neigh_indices, neigh_sqr_dists);
	// 	// add match only if the squared descriptor distance is less than max_descr_dist_ 
	// 	// SHOT descriptor distances are between 0 and 1 by design
	// 	if(found_neighs == 1 && neigh_sqr_dists[0] < (float)max_descr_dist_) 
	// 	{
	// 		pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);

	// 		// check if new correspondence is better than any previous at this point
	// 		bool found_better_result = false;
	// 		for (int j = 0; j < object_world_corrs->size(); ++j)
	// 		{
	// 			// is the found neigbor the same one like in the correspondence j
	// 			if (object_world_corrs->at(j).index_query == neigh_indices[0])
	// 			{
	// 				// do not add a new correspondence later
	// 				found_better_result = true;
	// 				// is the new distance smaller? (that means better)
	// 				if (neigh_sqr_dists[0] < object_world_corrs->at(j).distance)
	// 				{
	// 					// replace correspondence with better one
	// 					object_world_corrs->at(j) = corr;
	// 				}
	// 				else
	// 					// break out of inner loop to save time and try next keypoint
	// 					break;
	// 			}
	// 		}
	// 		// if this is a new correspondence, add a new correspondence at the end
	// 		if (!found_better_result)
	// 			object_world_corrs->push_back (corr);	
	// 	}
	// }
	// std::cout << "Correspondences found: " << object_world_corrs->size () << std::endl;


	// //
	// // all correspondences were found, now match the correspondences to the real object!
	// //
	// cluster(object_world_corrs);
}


//===================================================
//  world_descripter_fpfh_cb
// receive descr[pter for fpfh
//=================================================== 
void world_descriptor_FPFH_cb (const object_recognition::FPFH_bundle::Ptr input)
{
	FPFHfromROSMsg(*input, *world_descriptors_FPFH);
	ROS_INFO("world_descriptors_FPFH size: %d points", (int)world_descriptors_FPFH->size());
}

//===================================================
//  object_descriptor_FPFH_cb
// callback function when object descriptors received
// this will also trigger the recognition if all the 
// other keypoints and descriptors have been received
//=================================================== 
void object_descriptor_FPFH_cb (const object_recognition::FPFH_bundle::Ptr input)
{
	FPFHfromROSMsg(*input, *object_descriptors_FPFH);
	ROS_INFO("object FPFH Received %d descriptors ",(int)object_descriptors_FPFH->size());
}

//===================================================
//  shot352fromROSMsg
//  function to convert the custom ROS 
// message into a PCL descriptor type
//=================================================== 
void shot352fromROSMsg(const object_recognition::Shot352_bundle &input, DescriptorCloudShot352 &output)
{
	output.resize(input.descriptors.size());
	for (int j = 0 ; j < input.descriptors.size() ; ++j)
	{	
		std::copy(input.descriptors[j].descriptor.begin(), input.descriptors[j].descriptor.begin() + 352 , output[j].descriptor);
		std::copy(input.descriptors[j].rf.begin(), input.descriptors[j].rf.begin() + 9, output[j].rf);
	}
}

//===================================================
//  shot1344fromROSMsg
//  function to convert the custom ROS 
// message into a PCL descriptor type
//=================================================== 
void shot1344fromROSMsg(const object_recognition::Shot1344_bundle &input, DescriptorCloudShot1344 &output)
{
	output.resize(input.descriptors.size());
	for (int j = 0 ; j < input.descriptors.size() ; ++j)
	{	
		std::copy(input.descriptors[j].descriptor.begin(), input.descriptors[j].descriptor.begin() + 1344 , output[j].descriptor);
		std::copy(input.descriptors[j].rf.begin(), input.descriptors[j].rf.begin() + 9, output[j].rf);
	}
}

//===================================================
// FPFHfromROSMsg
//=================================================== 

void FPFHfromROSMsg(const object_recognition::FPFH_bundle &input, DescriptorCloudFPFH &output)
{
	output.resize(input.descriptors.size());
	for (int j = 0 ; j < input.descriptors.size() ; ++j)
		std::copy(input.descriptors[j].histogram.begin(), input.descriptors[j].histogram.begin() + 33 , output[j].histogram);
}


//===================================================
// cluster
// from the world correndence find a real object using
// cluster method. 
//=================================================== 
void cluster(const pcl::CorrespondencesPtr &object_world_corrs)
{
	ROS_INFO("cluster start ...");
	cg_size_ = 0.01;
	cg_thresh_ = 5.0;

	PointCloud correspondence_object;
	PointCloud correspondence_world;
	for (int j = 0; j < object_world_corrs->size (); ++j)
  {
		PointType& object_point = object_keypoints->at(object_world_corrs->at(j).index_query);
		PointType& world_point = world_keypoints->at(object_world_corrs->at(j).index_match);

		correspondence_object.push_back(object_point);
		correspondence_world.push_back(world_point);
  }

	// PointCloudROS pub_me_object2;
	// PointCloudROS pub_me_world2;
	// toROSMsg(correspondence_object, pub_me_object2);
	// toROSMsg(correspondence_world, pub_me_world2);
	// pub_me_object2.header.frame_id = "/object";
	// pub_me_world2.header.frame_id = "/world";
	// pub_object_corr.publish(pub_me_object2);
	// pub_world_corr.publish(pub_me_world2);

//   //
//   //  Actual Clustering
//   //
// 	cout << "... clustering ..." << endl;    
//   std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
//   std::vector<pcl::Correspondences> clustered_corrs;
    
// 	pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
// 	gc_clusterer.setGCSize (cg_size_);
// 	gc_clusterer.setGCThreshold (cg_thresh_);

// 	gc_clusterer.setInputCloud (object_keypoints);
// 	gc_clusterer.setSceneCloud (world_keypoints);
// 	gc_clusterer.setModelSceneCorrespondences (object_world_corrs);

// 	gc_clusterer.recognize (rototranslations, clustered_corrs);

    
//   //
//   //  Output results
//   //
//   std::cout << "Object instances found: " << rototranslations.size () << std::endl;
// 	int maximum = 0;
// 	int best;
// 	for (int i = 0; i < rototranslations.size (); ++i)
// 	{
// 		cout << "Instance "<< i << " has " << clustered_corrs[i].size () << " correspondences" << endl;
// 		if (maximum < clustered_corrs[i].size ())
// 		{
// 			maximum = clustered_corrs[i].size ();
// 			best = i;
// 		}
// 	}

//   if (rototranslations.size () > 0)
//   {
// 		cout << "selecting instance " << best << " and calculating TF" << endl;

//     // Print the rotation matrix and translation vector
//     Eigen::Matrix3f rotation = rototranslations[best].block<3,3>(0, 0);
//     Eigen::Vector3f translation = rototranslations[best].block<3,1>(0, 3);
    
//     printf ("\n");
//     printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
//     printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
//     printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
//     printf ("\n");
//     printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));

// 		// convert Eigen matricies into ROS TF message
// 		tf::Vector3 object_offset;
// 		tf::Quaternion object_rotation;
// 		object_offset[0] = translation (0);
// 		object_offset[1] = translation (1);
// 		object_offset[2] = translation (2);
// 		// convert rotation matrix to quaternion
// 		Eigen::Quaternionf quaternion (rotation);
// 		object_rotation[0] = quaternion.x();
// 		object_rotation[1] = quaternion.y();
// 		object_rotation[2] = quaternion.z();
// 		object_rotation[3] = quaternion.w();

// 		static tf::TransformBroadcaster br;
// 		tf::Transform transform;
// 		transform.setOrigin (object_offset);
// 		transform.setRotation (object_rotation);
// 		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "object"));

// 		//
// 		// Debug output
// 		//
// 		PointCloud correspondence_object_cluster;
// 		PointCloud correspondence_world_cluster;
		
// 		for (int j = 0; j < clustered_corrs[best].size (); ++j)
//     {
//       PointType& model_point = object_keypoints->at(clustered_corrs[best][j].index_query);
//       PointType& scene_point = world_keypoints->at(clustered_corrs[best][j].index_match);
// 			correspondence_object_cluster.push_back(model_point);
// 			correspondence_world_cluster.push_back(scene_point);
// 			//cout << model_point.x << " " << model_point.y << " " <<  model_point.z << endl;
// 			//cout << scene_point.x << " " <<  scene_point.y << " " <<  scene_point.z << endl;
//     }

// 		PointCloudROS pub_me_object;
// 		PointCloudROS pub_me_world;
// 		toROSMsg(correspondence_object_cluster, pub_me_object);
// 		toROSMsg(correspondence_world_cluster, pub_me_world);
// 		pub_me_object.header.frame_id = "/object";
// 		pub_me_world.header.frame_id = "/world";
// 		pub_object.publish(pub_me_object);
// 		pub_world.publish(pub_me_world);
//   }
}


	// //
	// //   region growing rgb way to segmetation
	// //
	// 	ros::Time Begin = ros::Time::now(); 
	// if(apply_segrgb)
	// {
	// 	// pcl::RegionGrowingRGB<PointType> reg_segrgb;
	// 	// std::vector <pcl::PointIndices> clusters_rgb;
	// 	// pcl::search::Search <PointType>::Ptr tree_segrgb = boost::shared_ptr<pcl::search::Search<PointType> > (new pcl::search::KdTree<PointType>);
	// 	// reg_segrgb.setInputCloud (world_keypoints);
	// 	// reg_segrgb.setSearchMethod (tree_segrgb);
	// 	// reg_segrgb.setDistanceThreshold (dist_thre);  //距离的阀值
	// 	// reg_segrgb.setPointColorThreshold (point_color_thre);  //点与点之间颜色容差
	// 	// reg_segrgb.setRegionColorThreshold (region_color_thre);  //区域之间容差
	// 	// reg_segrgb.setMinClusterSize (min_cluster);       //设置聚类的大小
	// 	// reg_segrgb.extract (clusters_rgb);	
	// 	// seg_num_rgb = clusters_rgb.size ();
	// 	// colored_cloud = reg_segrgb.getColoredCloud ();
	// 	// pcl::toROSMsg (*colored_cloud, *colored_rospc);
	// 	// colored_rospc->header.frame_id = "map";//map id for rviz i think 
	// 	// pub_world_color.publish(colored_rospc);

	// }
	// 	ros::Time Stop = ros::Time::now(); 
	// 	cout << " segmetation  ** get: " << seg_num_rgb << " ---------------Time :" << (Stop.toNSec() - Begin.toNSec())/1.0e9 << endl;
		

		// int r = 255, g = 0, b = 0;
		// int rgb = ((int)r) << 16 | ((int)g) << 8 | ((int)b);
		// (*aligned_pointcloud).points[0].rgb = rgb;
