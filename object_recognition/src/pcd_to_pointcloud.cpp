/*
 *
 *  Created on: Feb 27, 2013
 *      Author: Kai Franke
 */
#include "pcd_to_pointcloud.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "kinect_pointcloud");//ros init named pcd_descripter ..not node name (which the camkelists.txt do)
	ros::NodeHandle nh("~");
	//ros param server node callback 
    dynamic_reconfigure::Server<object_recognition::cconfigConfig> server;
    dynamic_reconfigure::Server<object_recognition::cconfigConfig>::CallbackType f;
    f = boost::bind(&paramCallback, _1, _2);
    server.setCallback(f);
	
	spincall = false;
	while(!spincall) 
	{
		ROS_WARN("choose your mode, wait for the start button ...");
		ros::spinOnce(); 
		ros::Duration(1).sleep();
	}

	if(apply_cam)
	{
		cout << "\n start to search a rgbd camera ..." << endl;
		// Declare pointcloud object, for calculating pointclouds and texture mappings
		pc_pointer = new rs2::pointcloud;
		// We want the points object to be persistent so we can display the last cloud when a frame drops
		points_pointer = new rs2::points;
		// // Declare RealSense pipeline, encapsulating the actual device
		pipe_point = new rs2::pipeline();
		// // Start streaming with default recommended configuration
		// Create a configuration for configuring the pipeline with a non default profile
		rs2::config cfg;
		//======================
		// Stream configuration
		//======================
		int frame_width = 640;
		int frame_height = 480;
		int frame_rate = 10;
		cfg.enable_stream(RS2_STREAM_COLOR, frame_width, frame_height, RS2_FORMAT_BGR8, frame_rate);
		cfg.enable_stream(RS2_STREAM_INFRARED, frame_width, frame_height, RS2_FORMAT_Y8, frame_rate);
		cfg.enable_stream(RS2_STREAM_DEPTH, frame_width, frame_height, RS2_FORMAT_Z16, frame_rate);
		
		// rs2::pipeline_profile selection = pipe.start(cfg); 
		pipe_point->start(cfg);
		cout << " success got one cam! start to rece a frame..." << endl<<endl;
		cout<<"** wait for frame settle **   \n";
		for (int i = 0; i < 30; i++) 
			auto frames = pipe_point->wait_for_frames(); //Drop several frames for auto-exposure
	}
	//
	// take care of the ROS stuff
	//

	// Create a ROS publisher for the output model coefficients 系数
	pub_points_full = nh.advertise<KeypointMsg> ("keypoints/points_full", 1);//advertise key point msg   //pub declare in head file 
	pub_points = nh.advertise<KeypointMsg> ("keypoints/points", 1);//advertise key point msg   //pub declare in head file 
	pub_points_ob =  nh.advertise<KeypointMsg> ("keypoints/points_ob", 1);
	pub_points_colora =  nh.advertise<KeypointMsg> ("keypoints/points_colora", 1);
	pub_keypoints = nh.advertise<KeypointMsg> ("keypoints/keypoints", 1);//advertise key point msg   //pub declare in head file 
	pub_keypoints_ob = nh.advertise<KeypointMsg> ("keypoints/keypoints_ob", 1);//advertise key point msg   //pub declare in head file 
	pub_descriptors_Shot352 = nh.advertise<Shot352Msg> ("descriptors/Shot352", 1);//advertise the descriptor msg same below
	pub_descriptors_Shot1344= nh.advertise<Shot1344Msg>("descriptors/Shot1344",1);
	pub_descriptors_FPFH = nh.advertise<FPFHMsg>("descriptors/FPFH", 1);

	// Create a ROS publisher for the output model coefficients 系数
	pub_descriptors_Shot352_ob = nh.advertise<Shot352Msg> ("descriptors/Shot352_ob", 1);//advertise the descriptor msg same below
	pub_descriptors_Shot1344_ob = nh.advertise<Shot1344Msg>("descriptors/Shot1344_ob",1);
	pub_descriptors_FPFH_ob = nh.advertise<FPFHMsg>("descriptors/FPFH_ob", 1);

	pub_points_vector = nh.advertise<PointVectorMsg>("points_vector",1);

	// create objects
	output_points_full = KeypointMsg::Ptr (new KeypointMsg); 
	output_points = KeypointMsg::Ptr (new KeypointMsg);	//get the object 实例 which the ptr declared in head file 
	output_keypoints = KeypointMsg::Ptr (new KeypointMsg);	//get the object 实例 which the ptr declared in head file 
	output_colora = KeypointMsg::Ptr (new KeypointMsg);	//get the object 实例 which the ptr declared in head file 
	output_descriptors_shot352 = Shot352Msg ::Ptr (new Shot352Msg );
	output_descriptors_shot1344= Shot1344Msg::Ptr (new Shot1344Msg);
	output_descriptors_FPFH = FPFHMsg::Ptr (new FPFHMsg);
	points_vector = PointVectorMsg::Ptr (new PointVectorMsg);

	//
	// create all neccessary objects 
	//
	cloud_r               			= PointCloud::Ptr     				(new PointCloud    ());//点云 
	cloud_a               			= PointCloud::Ptr     				(new PointCloud    ());//processed 点云 
	cloud_keypoints     			= PointCloud::Ptr     				(new PointCloud    ());//key point 
	cloud_normals       			= NormalCloud::Ptr					(new NormalCloud   ());//normal 
	cloud_descriptors_shot352 = DescriptorCloudShot352::Ptr	(new DescriptorCloudShot352());// acturally pcl's feature ptr 
	cloud_descriptors_shot1344 = DescriptorCloudShot1344::Ptr(new DescriptorCloudShot1344());// same as above
	cloud_descriptors_FPFH = DescriptorCloudFPFH::Ptr (new DescriptorCloudFPFH());          
	PointCloud::Ptr cloud_axised    = PointCloud::Ptr     				(new PointCloud    ());
	PointCloud::Ptr cloud_grid      = PointCloud::Ptr     				(new PointCloud    ());
	PointCloud::Ptr cloud_extract   = PointCloud::Ptr     				(new PointCloud    ());
	PointCloud::Ptr cloud_outlier   = PointCloud::Ptr     				(new PointCloud    ());
	PointCloud::Ptr cloud_groud     = PointCloud::Ptr     				(new PointCloud    ());

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr colored_cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr (new pcl::PointCloud<pcl::PointXYZRGBA> ());

	// Set up SAC parameters for plane segmentation  
	seg_plane.setOptimizeCoefficients (true);
	seg_plane.setModelType (pcl::SACMODEL_PLANE);
	seg_plane.setMethodType (pcl::SAC_RANSAC);
	seg_plane.setMaxIterations (1000);
	extract_planes.setNegative (true);	// Extract the found plane to remove the table
	pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);// construct coefficients for plane
	pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);	// constructor for point found as part of planar surface

  	pcl::search::Search <PointType>::Ptr tree_segrgb = boost::shared_ptr<pcl::search::Search<PointType> > (new pcl::search::KdTree<PointType>);
  	pcl::RegionGrowingRGB<PointType> reg_segrgb;
	std::vector <pcl::PointIndices> clusters_rgb;

	pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType> ());// normal estimation
	pcl::search::KdTree<PointType>::Ptr tree_fpfh (new pcl::search::KdTree<PointType> ());

	std::vector <pcl::PointIndices> clusters_rgb_deliver;
	seg_num = 0;

		ros::Time Begin = ros::Time::now();//use for time measure
		ros::Time Stop = ros::Time::now(); 
		ros::Time Begin_all = ros::Time::now();//use for all time measure
		ros::Time Stop_all = ros::Time::now(); 


	while(ros::ok()){

		if ((current_num != 0) && (!flag_model)) 
		{
			cout << "jump to seg_pieces" << endl;
			goto seg_pieces;
		}else cout << "not jump model" << endl;
		Begin_all = ros::Time::now();
		//
		// load the pcd into the point cloud
		//
		if(flag_model)
		{
			cout << "start to model mode... " << endl;
			string pcd_path = "/home/wang/catkin_work/src/object_recognition/obj/model/model" +to_string(pcd_name_model) + ".pcd";
			if (pcl::io::loadPCDFile (pcd_path, *cloud_r) == -1) //read cloud from pcd_path deliver from param  
			{
				PCL_ERROR ("Couldn't read file %s \n", pcd_path.c_str());
			}
			else cout<< "*****model*******read successivefully from pcd path************" <<endl
						<<"      apply_noplane default set to false for object"<<endl;
			
			seg_num = 0;
			current_num = 0;
			apply_noplane = false;
			apply_outlier = false;
			apply_segrgb = false;
		}
		else{
			if(apply_cam)
			{
				Begin = ros::Time::now();
			//trial
			cout << "start to cam mode ..." << endl;
			auto frames = pipe_point->wait_for_frames ();
			auto color = frames.get_color_frame ();

			if (!color)
			color = frames.get_infrared_frame ();
			// Tell pointcloud object to map to this color frame
			pc_pointer->map_to(color);
			auto depth = frames.get_depth_frame ();

			// Generate the pointcloud and texture mappings
			*points_pointer = pc_pointer->calculate(depth);
			cloud_r = PCL_Conversion(*points_pointer, color);//get PCL pointcloud type
				Stop = ros::Time::now(); 
				cout <<  " aliened the color and depth ++++++++++Time :" << (Stop.toNSec() - Begin.toNSec())/1.0e9<<endl;
			}else if(flag_world){
				cout << "start to world read mode ..." << endl;
				string pcd_path = "/home/wang/catkin_work/src/object_recognition/obj/world/world" +to_string(pcd_name) + ".pcd";
				if (pcl::io::loadPCDFile (pcd_path, *cloud_r) == -1) //read cloud from pcd_path deliver from param  
				{
					PCL_ERROR ("Couldn't read file %s \n", pcd_path.c_str());
				}
				else cout<< "***world*********read successivefully from pcd path************"<<endl;
			}else {
				ROS_WARN("not model used !!");
				ros::Duration(1).sleep();
				continue;
			}
		}

		//
		//  save pcd file from the origin pointcloud
		//
		if(flag_world_save)
		{
			string pcd_path = "/home/wang/catkin_work/src/object_recognition/obj/world/world" +to_string(pcd_name) + ".pcd";
			pcl::io::savePCDFileASCII(pcd_path, *cloud_r);
			std::cout << " SAVED FILE "<< to_string(pcd_name) << "cloud_r Cloud total points: " << cloud_r->size() << endl;
			flag_world_save = false;
		}


	//
	// apply the axis limits and grid down\ downscale the points and remove the plane
	//
			Begin = ros::Time::now();
		if(apply_axis)
		{
	// filter z values within a certain range set by parameters			
			double min = -axis_filter; 
			double max = axis_filter;
			// set parameters for z axis filtering
			pt.setInputCloud(cloud_r);
			pt.setKeepOrganized(keep_organized);
			pt.setFilterFieldName("x");
			pt.setFilterLimits(min, max);
			pt.filter(*cloud_axised);			
			pt.setInputCloud(cloud_axised);
			pt.setKeepOrganized(keep_organized);
			pt.setFilterFieldName("y");
			pt.setFilterLimits(min, max);
			pt.filter(*cloud_axised);
			cout << "after axis filter point size: "<< cloud_axised->size() << endl;
		}
		else pcl::copyPointCloud(*cloud_r,*cloud_axised);
		
		//
		//   grid
		//
		if(apply_voxel)
		{
			// Create the filtering object and downsample the dataset using the parameter leaf size
			sor_grid.setInputCloud (cloud_axised);
			sor_grid.setLeafSize (voxel_size,voxel_size,voxel_size);
			sor_grid.filter (*cloud_grid);
			cout << "after grid point size: "<< cloud_grid->size() << endl;
		}
		else pcl::copyPointCloud(*cloud_axised, *cloud_grid);
		
		//
		//   remove the plane
		//
		if(apply_noplane)
		{
			// set maximal distance from point to planar surface to be identified as plane
			seg_plane.setDistanceThreshold (threshold_plane);
			seg_plane.setInputCloud (cloud_grid); //maybe should set as the cloud_r with the remove part to keep mess 
			seg_plane.segment (*inliers_plane, *coefficients_plane);

			// remove plane from point cloud
			extract_planes.setInputCloud(cloud_grid);
			extract_planes.setIndices (inliers_plane);
			extract_planes.setKeepOrganized (keep_organized);
			extract_planes.filter (*cloud_extract);
			cout << "after noplane point size: "<< cloud_extract->size() << endl;
		}// else use remove ground way
		else if(apply_octree)
		{
			//load the ground
			if((!reset_octree_once) || reset_octree)
			{
				pcl::copyPointCloud(*cloud_grid, *cloud_groud);
				reset_octree_once = true;
				reset_octree = false;
			}

			pcl::octree::OctreePointCloudChangeDetector<PointType> octree (resolution);
			cout << "ground octree ... cloud_groud : " << cloud_groud->size () << " resolution: " << resolution << endl;
			octree.setInputCloud (cloud_groud);
			octree.addPointsFromInputCloud ();
			octree.switchBuffers ();
			cout << "compare change ... cloud_grid : " << cloud_grid->size () << endl;
			octree.setInputCloud (cloud_grid);
			octree.addPointsFromInputCloud ();
				std::vector<int> newPointIdxVector;
			// Get vector of point indices from octree voxels which did not exist in previous buffer
			octree.getPointIndicesFromNewVoxels (newPointIdxVector, noise_filter);
			cout << "  newPointIdxVector : "<< newPointIdxVector.size () << endl;

			if(newPointIdxVector.size () == 0) 
			{
				ROS_WARN("background detected, ground default deliver, please change input ...");
				pcl::copyPointCloud(*cloud_grid, *cloud_extract);
			}else{
				cloud_extract.reset (new PointCloud);
				for (std::size_t i = 0; i < newPointIdxVector.size (); ++i)
					cloud_extract->points.push_back(cloud_grid->points[newPointIdxVector[i]]);
				// cloud_extract->width = cloud_extract->size ();
				// cloud_extract->height = 1;
				cout << "after extra point size: "<< cloud_extract->size () << endl;
			}
		}
		else pcl::copyPointCloud(*cloud_grid, *cloud_extract);
		
		//
		//   remove the outlier by distance threshold
		//
		if(apply_outlier)
		{
			sor_outlier.setInputCloud (cloud_extract);
			sor_outlier.setMeanK (outlier_meanK);
			sor_outlier.setStddevMulThresh (outlier_Thresh);
			sor_outlier.filter (*cloud_a);
			cout << "after outlier point size: "<< cloud_a->size() << endl;
		}
		else pcl::copyPointCloud(*cloud_extract, *cloud_a);
		
			Stop = ros::Time::now(); 
			cout << " pre procession filter  ---------------Time :" << (Stop.toNSec() - Begin.toNSec())/1.0e9 << endl;
			

		// pcl::copyPointCloud(*cloud_outlier, *cloud_a);
		
		//
		//  Downsample cloud by extract keypoints,not grid
		//
			Begin = ros::Time::now();
		uniform_sampling_cloud.setInputCloud (cloud_a);//sameple down 
		uniform_sampling_cloud.setRadiusSearch (cloud_ss);
		uniform_sampling_cloud.compute (sampled_indices_cloud);
		pcl::copyPointCloud (*cloud_a, sampled_indices_cloud.points, *cloud_keypoints);// cout the down sample before and after be below
			Stop = ros::Time::now(); 
		cout <<  " extract kps cloud ... ----------------Time :" <<  (Stop.toNSec() - Begin.toNSec())/1.0e9<<endl;
		
// //debug mark 
// cloud_a --(sampled_indices_cloud.points) ---> cloud_keypoints
// cloud_a --(clusters_rgb[k].indices) ---> seg1\seg2\...

// //mention 
// clusters_rgb =! sampled_indices_cloud
// clusters_rgb is part of diviation but sampled_indices_cloud is smaple
		//
		//   region growing rgb way to segmetation
		//
		if(apply_segrgb)
		{
			if(current_num == 0)
			{
					Begin = ros::Time::now(); 
				reg_segrgb.setInputCloud (cloud_a);
				reg_segrgb.setSearchMethod (tree_segrgb);
				reg_segrgb.setDistanceThreshold (dist_thre);  //距离的阀值
				reg_segrgb.setPointColorThreshold (point_color_thre);  //点与点之间颜色容差 if points belong to the same region
				reg_segrgb.setRegionColorThreshold (region_color_thre);  //区域之间容差 if regions can be merged.
				reg_segrgb.setMinClusterSize (min_cluster);       //设置聚类的大小
				reg_segrgb.extract (clusters_rgb);	
				seg_num = clusters_rgb.size ();
				cout << "uniform_sampling_cloudstart to redeliver..." << endl;
				clusters_rgb_deliver.clear();
				pcl::PointIndices empty_index;
				for (int i = 0; i < clusters_rgb.size (); i++)
					clusters_rgb_deliver.push_back(empty_index);
				bool being_colored = false;
				PointCloud::iterator index = cloud_keypoints->begin();

				// core operation
				for (size_t i = 0; i < sampled_indices_cloud.points.size (); i++){
					being_colored = false;
					for (size_t j = 0; j < clusters_rgb.size (); j++){
						for (size_t k = 0; k < clusters_rgb[j].indices.size (); k++){
							if(clusters_rgb[j].indices[k] == sampled_indices_cloud.points[i]) 
							{
								clusters_rgb_deliver[j].indices.push_back (clusters_rgb[j].indices[k]);
								being_colored = true;
							}
							if(being_colored) break;
						}
					}
					// if (!being_colored) cloud_keypoints->erase(index + i);
				}

				// pcl::copyPointCloud (*cloud_keypoints, *cloud_outlier);
				current_num = seg_num;
					
					Stop = ros::Time::now(); 
				cout << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"<<endl;
				cout << " segmetation  ** get: " << seg_num << " ---------------Time :" << (Stop.toNSec() - Begin.toNSec())/1.0e9 << endl;
			}
			seg_pieces :
			if (current_num){
				pcl::copyPointCloud(*cloud_a, clusters_rgb_deliver[current_num-1], *cloud_keypoints);
				cout << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"<<endl;
				cout << "seg_pieces " << current_num << "/" << seg_num << " for :" << cloud_keypoints->size() << " points " << endl;
			}
		}else if (flag_world) seg_num = current_num = 1;


		//
		// compute normals   as shown, use the PLC lib to get  
		//
			Begin = ros::Time::now();
		norm_est_cloud.setSearchMethod (tree);
		// norm_est_cloud.setRadiusSearch (0.03);
		norm_est_cloud.setKSearch (8);//maybe this can be adjust 10
		norm_est_cloud.setNumberOfThreads(4);
		norm_est_cloud.setInputCloud (cloud_a);
		// norm_est_cloud.setInputCloud (cloud_keypoints);
		norm_est_cloud.compute (*cloud_normals);// compute normals
		   Stop = ros::Time::now(); 
		cout << " computed normals from cloud ----------Time :" << (Stop.toNSec() - Begin.toNSec())/1.0e9<<endl;

		//
		// Extract descriptors shot 352
		//
			Begin = ros::Time::now();
		if(apply_shot352)
		{

							string pcd_path_fph = "/home/wang/catkin_work/src/object_recognition/obj/kepoints.pcd";
							pcl::io::savePCDFile<PointType>(pcd_path_fph, *cloud_keypoints);
							cout<<"saved 0...\n";


							string pcd_path_fh = "/home/wang/catkin_work/src/object_recognition/obj/nomal.pcd";
							pcl::io::savePCDFile<NormalType>(pcd_path_fh, *cloud_normals);
							cout<<"saved 1...\n";


							string pcd_path_fp = "/home/wang/catkin_work/src/object_recognition/obj/cloud_a.pcd";
							pcl::io::savePCDFile<PointType>(pcd_path_fp, *cloud_a);
							cout<<"saved 2...\n";

			cout << "return the resolution of the keypoint needed to be computer :: " << computeCloudResolution (cloud_keypoints) << endl;
			descr_est_shot352.setInputCloud (cloud_keypoints);
			descr_est_shot352.setRadiusSearch (descr_rad);// the param from begin ,need to be adjust
			descr_est_shot352.setInputNormals (cloud_normals);// need the normal also 
			descr_est_shot352.setSearchSurface (cloud_a); //the surface that need transform  
			descr_est_shot352.compute (*cloud_descriptors_shot352);// output of shot 352 which not use color
			Shot352toROSMsg(*cloud_descriptors_shot352, *output_descriptors_shot352);// shot352 to ROSmsg -- features// same below ,but here should take pcl::roROSmsg?
			
			//debug code 
			int nan_num = 0;
			for (size_t i = 0; i < cloud_descriptors_shot352->size (); ++i)
			{
				if (!pcl_isfinite (cloud_descriptors_shot352->at(i).descriptor[0])) //skipping NaNs
					nan_num ++;						
			}
			ROS_WARN("total  num : %d/%d  rate is : %f ",  nan_num, (int)cloud_descriptors_shot352->size(),\
			(float)(nan_num*100.0)/(float)cloud_descriptors_shot352->size());
		}

		//
		//   Extract descriptors shot1344
		//
		if(apply_shot1344)
		{
			descr_est_shot1344.setInputCloud (cloud_keypoints);//same as above 
			descr_est_shot1344.setRadiusSearch (descr_rad);
			descr_est_shot1344.setInputNormals (cloud_normals);
			descr_est_shot1344.setSearchSurface (cloud_a);
			descr_est_shot1344.compute (*cloud_descriptors_shot1344); //but this is from color ,which i think need to make a justice which one for
			Shot1344toROSMsg(*cloud_descriptors_shot1344, *output_descriptors_shot1344);//maybe because of the format need tobe reload this fun
						
			//debug code 
			int nan_num = 0;
			for (size_t i = 0; i < cloud_descriptors_shot1344->size (); ++i)
			{
				if (!pcl_isfinite (cloud_descriptors_shot1344->at(i).descriptor[0])) //skipping NaNs
					nan_num ++;						
			}
			ROS_WARN("total  num : %d/%d  rate is : %f ",  nan_num, (int)cloud_descriptors_shot1344->size(),\
			(float)(nan_num*100.0)/(float)cloud_descriptors_shot1344->size());
		}
			
		//
		//   Extract descriptors fpfh
		//
		if(apply_fpfh)
		{
			//maybe nomral are nan
			descr_est_fpfh.setNumberOfThreads(4); //指定4核计算
			descr_est_fpfh.setInputCloud(cloud_a);
			// descr_est_fpfh.setInputCloud(cloud_keypoints);
			descr_est_fpfh.setInputNormals(cloud_normals);
			descr_est_fpfh.setSearchMethod(tree_fpfh);
			descr_est_fpfh.setKSearch(10);//10

			descr_est_fpfh.compute(*cloud_descriptors_FPFH);
			//debug

			if ((current_num) && apply_segrgb)
				pcl::copyPointCloud(*cloud_descriptors_FPFH, clusters_rgb_deliver[current_num - 1], *cloud_descriptors_FPFH);
			else pcl::copyPointCloud (*cloud_descriptors_FPFH, sampled_indices_cloud.points, *cloud_descriptors_FPFH);
			// if(flag_model) pcl::copyPointCloud (*cloud_descriptors_FPFH, sampled_indices_cloud.points, *cloud_descriptors_FPFH);

            cout << "compute about :" << cloud_descriptors_FPFH->size() << " fpfh descriptors" << endl;
			// ROS_WARN("total  num : %d ", (int)cloud_descriptors_FPFH->size());
			FPFHtoROSMsg(*cloud_descriptors_FPFH, *output_descriptors_FPFH);
		}
			Stop = ros::Time::now(); 
		cout << " extracted descriptors from cloud -----Time :" <<(Stop.toNSec() - Begin.toNSec())/1.0e9 << endl;
		
		//
		//   turn the cloud to rosmsg
		//
		pcl::toROSMsg(*cloud_axised, *output_points_full);
		pcl::toROSMsg(*cloud_a, *output_points);		
		pcl::toROSMsg(*cloud_keypoints, *output_keypoints);
		// pcl::toROSMsg(*colored_cloud, *output_colora);
		//
		//   publish descriptors pointcloud 
		//
		output_points->header.frame_id = output_frame;//map id for rviz i think 
		output_points_full->header.frame_id = output_frame;//map id for rviz i think 
		output_keypoints->header.frame_id = output_frame;//map id for rviz i think 
		// output_colora->header.frame_id = output_frame;

		pub_points_full.publish(*output_points_full);
		if (ros::ok()) 
		{ 
			if(flag_world)
			{ 
			// Publish Keypoints and Descriptors for world
				pub_points.publish(*output_points); // if there is sub , pub the output_points 
				pub_keypoints.publish(*output_keypoints); // if there is sub , pub the keypoint 
				// pub_points_colora.publish(*output_colora); // if there is sub , pub the keypoint 
				if(apply_shot352)  pub_descriptors_Shot352.publish(*output_descriptors_shot352); // pub the feature also,shot352
				if(apply_shot1344) pub_descriptors_Shot1344.publish(*output_descriptors_shot1344);// pub shot1344
				if(apply_fpfh) pub_descriptors_FPFH.publish(*output_descriptors_FPFH);
								// next to publish the range
				points_vector->x = seg_num;
				points_vector->y = current_num;
				points_vector->z = cloud_keypoints->size ();
				pub_points_vector.publish (*points_vector);
				current_num --;

			}else if(flag_model){
			// Publish Keypoints and Descriptors for model
				pub_points_ob.publish(*output_points); // if there is sub , pub the output_points 
				pub_keypoints_ob.publish(*output_keypoints); // if there is sub , pub the keypoint 
				if(apply_shot352)  pub_descriptors_Shot352_ob.publish(*output_descriptors_shot352); // pub the feature also,shot352
				if(apply_shot1344) pub_descriptors_Shot1344_ob.publish(*output_descriptors_shot1344);// pub shot1344
				if(apply_fpfh) pub_descriptors_FPFH_ob.publish(*output_descriptors_FPFH);
			}
		}

		Stop_all = ros::Time::now(); 
		//print the feature size
		std::cout <<" ** feature Rresult ** "<< endl << "Cloud total points: " << cloud_a->size () 
			<< "; Selected Keypoints: " << cloud_keypoints->size () << std::endl
			<< "Pubbed keypoint and shot   "
			<<"--- shot352:"<< (*output_descriptors_shot352).descriptors.size()
			<<" // shot1344:"  << (*output_descriptors_shot1344).descriptors.size()
			<<" // FPFH:" << (*output_descriptors_FPFH).descriptors.size()<< endl
			<<"frame rate : "<< 1.0e9/ (Stop_all.toNSec() - Begin_all.toNSec())<<" Hz"
			<<endl<<"--------------- end ---------------------------------"
			<< endl<< endl<< endl<< endl<<"============== start ====================================== " 
			<< endl;
			
		ros::spinOnce(); 

		while(!spincall) 
		{
			ROS_WARN("process is ready, wait for start ...");
			ros::spinOnce(); 
			ros::Duration(1).sleep();
		}
		if(ShutDown) return 0;			
	}
	return 0;
}


//declare the conver fun of shot352 to rosmsg

// float64[352] descriptor
// float64[9] rf

void Shot352toROSMsg(const DescriptorCloudShot352 &input, Shot352Msg &output)
{
	output.descriptors.resize(input.size());
	for (int j = 0 ; j < input.size() ; ++j)
	{	
		std::copy(input[j].descriptor, input[j].descriptor + 352 , output.descriptors[j].descriptor.begin());
		std::copy(input[j].rf, input[j].rf + 9 , output.descriptors[j].rf.begin());
	}
}

//declare the conver fun of shot1344 to rosmsg
void Shot1344toROSMsg(const DescriptorCloudShot1344 &input, Shot1344Msg &output)
{
	output.descriptors.resize(input.size());
	for (int j = 0 ; j < input.size() ; ++j)
	{	
		std::copy(input[j].descriptor, input[j].descriptor + 1344 , output.descriptors[j].descriptor.begin());
		std::copy(input[j].rf, input[j].rf + 9 , output.descriptors[j].rf.begin());
	}
}

void FPFHtoROSMsg(const DescriptorCloudFPFH &input, FPFHMsg &output)
{
	output.descriptors.resize(input.size());
	for (int j = 0 ; j < input.size() ; ++j)
		std::copy(input[j].histogram, input[j].histogram + 33 , output.descriptors[j].histogram.begin());
}


//======================================================
// RGB Texture
// - Function is utilized to extract the RGB data from
// a single point return R, G, and B values. 
// Normals are stored as RGB components and
// correspond to the specific depth (XYZ) coordinate.
// By taking these normals and converting them to
// texture coordinates, the RGB components can be
// "mapped" to each individual point (XYZ).
//======================================================
std::tuple<int, int, int> RGB_Texture(rs2::video_frame texture, rs2::texture_coordinate Texture_XY)
{
    // Get Width and Height coordinates of texture
    int width  = texture.get_width();  // Frame width in pixels
    int height = texture.get_height(); // Frame height in pixels
    
    // Normals to Texture Coordinates conversion
    int x_value = min(max(int(Texture_XY.u * width  + .5f), 0), width - 1);
    int y_value = min(max(int(Texture_XY.v * height + .5f), 0), height - 1);

    int bytes = x_value * texture.get_bytes_per_pixel();   // Get # of bytes per pixel
    int strides = y_value * texture.get_stride_in_bytes(); // Get line width in bytes
    int Text_Index =  (bytes + strides);

    const auto New_Texture = reinterpret_cast<const uint8_t*>(texture.get_data());
    
    // RGB components to save in tuple
    int NT1 = New_Texture[Text_Index];
    int NT2 = New_Texture[Text_Index + 1];
    int NT3 = New_Texture[Text_Index + 2];

    return std::tuple<int, int, int>(NT1, NT2, NT3);
}

//===================================================
//  PCL_Conversion
// - Function is utilized to fill a point cloud
//  object with depth and RGB data from a single
//  frame captured using the Realsense.
//=================================================== 
PointCloud::Ptr PCL_Conversion(const rs2::points& points, const rs2::video_frame& color){
    //input param is points and RGB ,combine it to return 

    // Object Declaration (Point Cloud)
    PointCloud::Ptr cloud(new PointCloud);

    // Declare Tuple for RGB value Storage (<t0>, <t1>, <t2>)
    std::tuple<uint8_t, uint8_t, uint8_t> RGB_Color;

    //================================
    // PCL Cloud Object Configuration
    //================================
    // Convert data captured from Realsense camera to Point Cloud
    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    
    cloud->width  = static_cast<uint32_t>( sp.width()  );   
    cloud->height = static_cast<uint32_t>( sp.height() );
    cloud->is_dense = false;
    cloud->points.resize( points.size() );

    auto Texture_Coord = points.get_texture_coordinates();
    auto Vertex = points.get_vertices();

    // Iterating through all points and setting XYZ coordinates
    // and RGB values
    for (int i = 0; i < points.size(); i++)
    {   
        //===================================
        // Mapping Depth Coordinates
        // - Depth data stored as XYZ values
        //===================================
        cloud->points[i].x = Vertex[i].x;
        cloud->points[i].y = Vertex[i].y;
        cloud->points[i].z = Vertex[i].z;

        // Obtain color texture for specific point
        RGB_Color = RGB_Texture(color, Texture_Coord[i]);

        // Mapping Color (BGR due to Camera Model)
        cloud->points[i].r = get<2>(RGB_Color); // Reference tuple<2>
        cloud->points[i].g = get<1>(RGB_Color); // Reference tuple<1>
        cloud->points[i].b = get<0>(RGB_Color); // Reference tuple<0>
    }
    
   return cloud; // PCL RGB Point Cloud generated
}


//===================================================
//  computeCloudResolution
// - Function computer a point cloud resolution
//=================================================== 
double
computeCloudResolution (const pcl::PointCloud<PointType>::ConstPtr &cloud)
{
  double res = 0.0;
  int n_points = 0;
  int nres;
  std::vector<int> indices (2);
  std::vector<float> sqr_distances (2);
  pcl::search::KdTree<PointType> tree;
  tree.setInputCloud (cloud);

  for (std::size_t i = 0; i < cloud->size (); ++i)
  {
    if (! std::isfinite ((*cloud)[i].x))
    {
      continue;
    }
    //Considering the second neighbor since the first is the point itself.
    nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
    if (nres == 2)
    {
      res += sqrt (sqr_distances[1]);
      ++n_points;
    }
  }
  if (n_points != 0)
  {
    res /= n_points;
  }
  return res;
}

//
//   paramcallback funct for dynamic cfg service
//
void paramCallback(object_recognition::cconfigConfig &config, uint32_t level) {

	apply_cam = config.apply_cam;
	spincall = config.start;
	ShutDown = config.CLOSE;
	cloud_ss = config.cloud_ss;
	descr_rad = config.descr_rad;
	strcpy(output_frame,config.output_frame.c_str());

	apply_voxel = config.apply_voxel;
	voxel_size = config.voxel_size;

	apply_noplane = config.apply_noplane;
	keep_organized = config.keep_organized;
	threshold_plane = config.threshold_plane;

	apply_outlier = config.apply_outlier;
	outlier_meanK = config.outlier_meanK;
	outlier_Thresh = config.outlier_Thresh;

	apply_shot352 = config.apply_shot352;
	apply_shot1344 = config.apply_shot1344;

	flag_model = config.flag_model;//
	flag_world = config.flag_world;//
	flag_world_save = config.flag_world_save;//

	pcd_name = config.pcd_name;
	pcd_name_model = config.pcd_name_model;

	apply_axis = config.apply_axis;
	axis_filter = config.axis_filter;
	apply_fpfh = config.apply_fpfh;

	apply_octree = config.apply_octree;
	reset_octree = config.reset_octree;
	resolution = config.resolution;
	noise_filter = config.noise_filter;

	apply_segrgb = config.apply_segrgb;
	dist_thre = config.dist_thre;
	point_color_thre = config.point_color_thre;
	region_color_thre = config.region_color_thre;
	min_cluster = config.min_cluster;


	if(flag_world) flag_model = false;
	if(apply_octree) apply_noplane = false;

	if(flag_world_save) {
		if (!apply_cam){
			ROS_WARN("cam not open !!! set model mode");
			flag_world_save = false;
			flag_model = true;
			return;
		}
		flag_model = false;
		flag_world = false;
		apply_noplane = false;
		apply_outlier = false;
		apply_octree = false;
		apply_segrgb = false;
	}

	ROS_WARN(" Reconfigure server get :\n\
	cloud_ss:%f,  descr_rad:%f,  output_frame:%s,  \n\
	apply_voxel:%s  voxel_size:%f,  \n\
	apply_noplane:%s  keep_organized:%s  threshold_plane:%f,\n\
	apply_outlier:%s  outlier_meanK:%d,  outlier_Thresh:%f,\n", cloud_ss,  descr_rad,  output_frame,
	apply_voxel?"True":"False",  voxel_size,  apply_noplane?"True":"False", keep_organized?"True":"False",  threshold_plane,
	apply_outlier?"True":"False",  outlier_meanK,  outlier_Thresh);
}


		//
		// Convert to ROS message
		// pcl::PCLPointCloud2::Ptr pclpc2 = pcl::PCLPointCloud2::Ptr (new pcl::PCLPointCloud2);
		// pcl::toPCLPointCloud2(*cloud_a, *pclpc2); //conver to the rosmsg -- keypoint
		// pcl_conversions::fromPCL(*pclpc2, *output_keypoints); //conver to the rosmsg -- keypoint
					
				// cout << "read the ground file ..." << endl;
				// string pcd_path_ground = "/home/wang/catkin_work/src/object_recognition/obj/ground/world11.pcd";
				// if (pcl::io::loadPCDFile (pcd_path_ground, *cloud_groud) == -1) //read cloud from pcd_path deliver from param  
				// {
				// 	PCL_ERROR ("Couldn't read file %s \n", pcd_path_ground.c_str());
				// }

							// string pcd_path_fpfh = "/home/wang/catkin_work/src/object_recognition/obj/fpfh.pcd";
							// pcl::io::savePCDFile<FPFH>(pcd_path_fpfh, *cloud_descriptors_FPFH);
							// cout<<"saved fpfh...\n";


        // viewer->addPointCloud<PointType>(cloud_r,"sample cloud");
		// viewer->addPointCloudNormals<PointType, NormalType>(cloud_a, cloud_normals,10,0.05,"normal");
		// viewer->addCoordinateSystem(1.0);
		// viewer->spinOnce(100);

	//   header: 
	//   seq: 0 stamp: 0 frame_id: 
	//   indices[]
	//   indices[0]:   0
	//   indices[1]:   1
	//  The RGBA information is available either as separate r, g, b, or as a
    //  * packed std::uint32_t rgba value. To pack it, use:
    //  *
    //  * \code
	//  * int rgba =  ((int)clusters_rgb_deliver.size ()) << 24 | (int)rgb ;
    //  * int rgb = ((int)r) << 16 | ((int)g) << 8 | ((int)b);
    //  * \endcode
    //  *
    //  * To unpack it use:
    //  *
    //  * \code
    //  * int rgb = ...;
    //  * std::uint8_t r = (rgb >> 16) & 0x0000ff;
    //  * std::uint8_t g = (rgb >> 8)  & 0x0000ff;
    //  * std::uint8_t b = (rgb)     & 0x0000ff;
    //  * \endcode

			// for (size_t j = 0; j < clusters_rgb_deliver.size (); j++)
			// 	cout << " colored_cloud->size() : " << colored_cloud->size() << "clusters_rgb_deliver " \
			// 	<< j << " has size : " << clusters_rgb_deliver[j].indices.size () << endl;


	// pcl::visualization::CloudViewer viewer ("Simple Viewer");
    // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));  //设置一个boost共享对象，并分配内存空间
	// viewer->setBackgroundColor(0.0,0,0);

				// colored_cloud = reg_segrgb.getColoredCloud ();			
				// colored_cloud.reset (new pcl::PointCloud <pcl::PointXYZRGBA>);
				// std::vector <pcl::PointIndices> clusters_rgb_deliver (clusters_rgb.size ());
								// pcl::PointXYZRGBA point_temp;
								// point_temp.x = cloud_keypoints->points[i].x;
								// point_temp.y = cloud_keypoints->points[i].y;
								// point_temp.z = cloud_keypoints->points[i].z;
								// point_temp.rgba =  ((int)j) << 24 | ((int)cloud_keypoints->points[i].rgb);
								// colored_cloud->push_back (point_temp);
								// cout << "adding point to colorpointcloud ... now size :" << colored_cloud->size();
				// for (size_t j = 0; j < clusters_rgb_deliver.size (); j++)
				// 	cout << " colored_cloud->size() : " << colored_cloud->size() << "  clusters_rgb_deliver " \
				// 		<< j << " has size : " << clusters_rgb_deliver[j].indices.size () << endl;
				// for (size_t i = 0; i < colored_cloud->size(); i ++)
				// 	cout << (int)((int)colored_cloud->points[i].rgba >> 24);
				// extract_planes.setInputCloud(cloud_outlier);
				// // extract_planes.setIndices (clusters_rgb_deliver[--current_num]);
				// extract_planes.setIndices (clusters_rgb_deliver.begin());
				// extract_planes.setKeepOrganized (keep_organized);
				// extract_planes.filter (*cloud_keypoints);