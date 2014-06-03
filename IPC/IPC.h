/*
 * IPC.h
 *
 *  Created on: 05.05.2014
 *      Author: Georg Semmler
 */

#ifndef IPC_H_
#define IPC_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <string>
#include <iostream>

#include <cmath>

#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

// Types
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT, PointNT, FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

namespace Calculation
{

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

bool runIPC(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr initial,
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr target,
		Eigen::Matrix4f& transformation, bool debug = false)
{
	//pcl::IterativeClosestPointNonLinear<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icp;
	pcl::IterativeClosestPoint<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icp;
	pcl::PointCloud<pcl::PointXYZRGBNormal> Final;
	bool ret = false;
	if (debug)
		setVerbosityLevel(pcl::console::L_DEBUG);
	std::cout << "init" << std::endl;
	//pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr iPtr(initial);
	//pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr fPtr(target);
	std::cout << "main loop" << std::endl;
	//icp.setMaxCorrespondenceDistance (0.1);
	int counter = 0;
	//float score=0;
	//do{
	icp.setMaximumIterations(200);
	icp.setInputSource(initial);
	icp.setInputTarget(target);
	std::cout << "align" << std::endl;
	icp.align(Final);
	std::cout << " score: " << icp.getFitnessScore() << std::endl;
	/*if(abs(score-icp.getFitnessScore())<pow10(-1*(counter+2))&&score!=0){
	 std::cout<<"does not converge"<<std::endl;
	 break;
	 }*/
	//score=icp.getFitnessScore();
	//}while(counter<4&&icp.getFitnessScore()>1e-7);
	transformation = icp.getFinalTransformation();
	if (icp.getFitnessScore() < 0.00000001) {
		std::cout << "converge" << std::endl;
		std::cout << "transform: " << std::endl << transformation << std::endl;
		ret = true;
	} else {
		std::cout << "does not converge" << std::endl;
	}
	std::cout << "befor return" << std::endl;
	return ret;
}

// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation: public pcl::PointRepresentation<PointNormalT>
{
	using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
	MyPointRepresentation()
	{
		// Define the number of dimensions
		nr_dimensions_ = 4;
	}

	// Override the copyToFloatArray method to define our feature vector
	virtual void copyToFloatArray(const PointNormalT &p, float * out) const
	{
		// < x, y, z, curvature >
		out[0] = p.x;
		out[1] = p.y;
		out[2] = p.z;
		out[3] = p.curvature;
	}
};

/*void showCloudsRight(const PointCloudWithNormals::Ptr cloud_target,
 const PointCloudWithNormals::Ptr cloud_source)
 {
 //p->removePointCloud("source");
 //p->removePointCloud("target");

 PointCloudColorHandlerGenericField<PointNormalT> tgt_color_handler(
 cloud_target, "curvature");
 if (!tgt_color_handler.isCapable())
 PCL_WARN("Cannot create curvature color handler!");

 PointCloudColorHandlerGenericField<PointNormalT> src_color_handler(
 cloud_source, "curvature");
 if (!src_color_handler.isCapable())
 PCL_WARN("Cannot create curvature color handler!");

 p->addPointCloud(cloud_target, tgt_color_handler, "target", vp_2);
 p->addPointCloud(cloud_source, src_color_handler, "source", vp_2);

 p->spinOnce();
 }*/

void pairAlign(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt,
		PointCloud::Ptr output, Eigen::Matrix4f &final_transform,
		bool downsample = false)
{

	//
	// Downsample for consistency and speed
	// \note enable this for large datasets
	std::cout << "geht" << std::endl;
	PointCloud::Ptr src(new PointCloud);
	PointCloud::Ptr tgt(new PointCloud);
	pcl::VoxelGrid<PointT> grid;
	if (downsample) {
		grid.setLeafSize(0.05, 0.05, 0.05);
		grid.setInputCloud(cloud_src);
		grid.filter(*src);

		grid.setInputCloud(cloud_tgt);
		grid.filter(*tgt);
	} else {
		src = cloud_src;
		tgt = cloud_tgt;
	}
	std::cout << "geht" << std::endl;
	// Compute surface normals and curvature
	PointCloudWithNormals::Ptr points_with_normals_src(
			new PointCloudWithNormals);
	PointCloudWithNormals::Ptr points_with_normals_tgt(
			new PointCloudWithNormals);

	pcl::NormalEstimation<PointT, PointNormalT> norm_est;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
			new pcl::search::KdTree<pcl::PointXYZ>());
	norm_est.setSearchMethod(tree);
	norm_est.setKSearch(30);

	norm_est.setInputCloud(src);
	norm_est.compute(*points_with_normals_src);
	pcl::copyPointCloud(*src, *points_with_normals_src);

	norm_est.setInputCloud(tgt);
	norm_est.compute(*points_with_normals_tgt);
	pcl::copyPointCloud(*tgt, *points_with_normals_tgt);
	std::cout << "input points: " << points_with_normals_src->size()
			<< std::endl;
	std::cout << "target points: " << points_with_normals_tgt->size()
			<< std::endl;
	//
	// Instantiate our custom point representation (defined above) ...
	MyPointRepresentation point_representation;
	// ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
	float alpha[4] = { 1.0, 1.0, 1.0, 1.0 };
	point_representation.setRescaleValues(alpha);

	//
	// Align
	pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
	reg.setTransformationEpsilon(1e-6);
	// Set the maximum distance between two correspondences (src<->tgt) to 10cm
	// Note: adjust this based on the size of your datasets
	//reg.setMaxCorrespondenceDistance(0.1);
	// Set the point representation
	reg.setPointRepresentation(
			boost::make_shared<const MyPointRepresentation>(
					point_representation));

	reg.setInputSource(points_with_normals_src);
	reg.setInputTarget(points_with_normals_tgt);

	//
	// Run the same optimization in a loop and visualize the results
	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource;
	PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
	reg.setMaximumIterations(2);
	for (int i = 0; i < 30; ++i) {
		PCL_INFO("Iteration Nr. %d.\n", i);

		// save cloud for visualization purpose
		points_with_normals_src = reg_result;

		// Estimate
		reg.setInputSource(points_with_normals_src);
		reg.align(*reg_result);

		//accumulate transformation between each Iteration
		Ti = reg.getFinalTransformation() * Ti;

		//if the difference between this transformation and the previous one
		//is smaller than the threshold, refine the process by reducing
		//the maximal correspondence distance
		if (fabs((reg.getLastIncrementalTransformation() - prev).sum())
				< reg.getTransformationEpsilon())
			reg.setMaxCorrespondenceDistance(
					reg.getMaxCorrespondenceDistance() - 0.001);

		prev = reg.getLastIncrementalTransformation();

		// visualize current state
		//showCloudsRight(points_with_normals_tgt, points_with_normals_src);
	}

	//
	// Get the transformation from target to source
	targetToSource = Ti.inverse();
	std::cout << "score " << reg.getFitnessScore() << std::endl;

	//
	// Transform target back in source frame
	pcl::transformPointCloud(*cloud_tgt, *output, targetToSource);

	/*p->removePointCloud("source");
	 p->removePointCloud("target");

	 PointCloudColorHandlerCustom<PointT> cloud_tgt_h(output, 0, 255, 0);
	 PointCloudColorHandlerCustom<PointT> cloud_src_h(cloud_src, 255, 0, 0);
	 p->addPointCloud(output, cloud_tgt_h, "target", vp_2);
	 p->addPointCloud(cloud_src, cloud_src_h, "source", vp_2);

	 PCL_INFO("Press q to continue the registration.\n");
	 p->spin();

	 p->removePointCloud("source");
	 p->removePointCloud("target");*/

	//add the source to the transformed target
	*output += *cloud_src;

	final_transform = targetToSource;
}

void test(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input,
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr target)
{

	FeatureCloudT::Ptr object_features(new FeatureCloudT);
	FeatureCloudT::Ptr scene_features(new FeatureCloudT);
	PointCloudT::Ptr object(new PointCloudT);
	PointCloudT::Ptr object_aligned(new PointCloudT);
	PointCloudT::Ptr scene(new PointCloudT);

	for(auto p:input->points){
		PointNT pn;
		pn.x=p.x;
		pn.y=p.y;
		pn.z=p.z;
		pn.normal_x=p.normal_x;
		pn.normal_y=p.normal_y;
		pn.normal_z=p.normal_z;
		pn.curvature=p.curvature;
		scene->push_back(pn);
	}

	for(auto p:target->points){
			PointNT pn;
			pn.x=p.x;
			pn.y=p.y;
			pn.z=p.z;
			pn.normal_x=p.normal_x;
			pn.normal_y=p.normal_y;
			pn.normal_z=p.normal_z;
			pn.curvature=p.curvature;
			object->push_back(pn);
		}

	pcl::console::print_highlight("Downsampling...\n");
	pcl::VoxelGrid<PointNT> grid;
	const float leaf = 0.005f;
	grid.setLeafSize(leaf, leaf, leaf);
	grid.setInputCloud(object);
	grid.filter(*object);
	grid.setInputCloud(scene);
	grid.filter(*scene);
	// Estimate normals for scene
	pcl::console::print_highlight("Estimating scene normals...\n");
	pcl::NormalEstimationOMP<PointNT, PointNT> nest;
	nest.setRadiusSearch(0.01);
	nest.setInputCloud(scene);
	nest.compute(*scene);

	// Estimate features
	pcl::console::print_highlight("Estimating features...\n");
	FeatureEstimationT fest;
	fest.setRadiusSearch(0.025);
	fest.setInputCloud(object);
	fest.setInputNormals(object);
	fest.compute(*object_features);
	fest.setInputCloud(scene);
	fest.setInputNormals(scene);
	fest.compute(*scene_features);

	// Perform alignment
	pcl::console::print_highlight("Starting alignment...\n");
	pcl::SampleConsensusPrerejective<PointNT, PointNT, FeatureT> align;
	align.setInputSource(object);
	align.setSourceFeatures(object_features);
	align.setInputTarget(scene);
	align.setTargetFeatures(scene_features);
	align.setNumberOfSamples(3); // Number of points to sample for generating/prerejecting a pose
	align.setCorrespondenceRandomness(2); // Number of nearest features to use
	align.setSimilarityThreshold(0.6f); // Polygonal edge length similarity threshold
	align.setMaxCorrespondenceDistance(1.5f * leaf); // Set inlier threshold
	align.setInlierFraction(0.25f); // Set required inlier fraction
	align.align(*object_aligned);

	if (align.hasConverged()) {
		// Print results
		Eigen::Matrix4f transformation = align.getFinalTransformation();
		pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n",
				transformation(0, 0), transformation(0, 1),
				transformation(0, 2));
		pcl::console::print_info("R = | %6.3f %6.3f %6.3f | \n",
				transformation(1, 0), transformation(1, 1),
				transformation(1, 2));
		pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n",
				transformation(2, 0), transformation(2, 1),
				transformation(2, 2));
		pcl::console::print_info("\n");
		pcl::console::print_info("t = < %0.3f, %0.3f, %0.3f >\n",
				transformation(0, 3), transformation(1, 3),
				transformation(2, 3));
		pcl::console::print_info("\n");
		pcl::console::print_info("Inliers: %i/%i\n", align.getInliers().size(),
				object->size());

		// Show alignment
		pcl::visualization::PCLVisualizer visu("Alignment");
		visu.addPointCloud(scene, ColorHandlerT(scene, 0.0, 255.0, 0.0),
				"scene");
		visu.addPointCloud(object_aligned,
				ColorHandlerT(object_aligned, 0.0, 0.0, 255.0),
				"object_aligned");
		visu.spin();
		runIPC(input,target,transformation,true);
	} else {
		pcl::console::print_error("Alignment failed!\n");
	}
}
}

#endif /* IPC_H_ */
