/*
 * IPC.h
 *
 *  Created on: 05.05.2014
 *      Author: Georg Semmler
 */

#ifndef IPC_H_
#define IPC_H_

#include <string>
#include <iostream>

#include <cmath>

#include <boost/make_shared.hpp>
#include <Eigen/Core>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/registration/sample_consensus_prerejective.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/console/print.h>

#include <pcl/segmentation/sac_segmentation.h>


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

/**
 * Wrapperclass for Iterative Closest Point Calculations
 */
class ICP{
public:
/**
 * Calculate the transformation between two point clouds using icp
 * WARNING: Only use this if the point clouds are near
 *
 * @param initial The initial point cloud
 * @param target The target point cloud
 * @param debug flag to specify the debug level
 *         - 0 no Debug log
 *         - 1 showing to clouds in viewer
 *         - >1 showing pcl debuglog
 * @param final The final aligned pointcloud
 *
 * @return the transformation
 *
 */
const static Eigen::Matrix4f runICP(PointCloudT::Ptr initial, PointCloudT::Ptr target,
		short debug = 0, PointCloudT::Ptr final = nullptr)
{
	pcl::IterativeClosestPoint<PointNT, PointNT> icp;
	PointCloudT Final;
	if (debug > 1) {
		setVerbosityLevel(pcl::console::L_DEBUG);
	}
	icp.setInputSource(initial);
	icp.setMaximumIterations(200);
	icp.setInputTarget(target);
	icp.align(Final);
	if (final != nullptr) {
		final->clear();
		for (const auto& p : Final) {
			final->push_back(p);
		}
	}
	if (debug > 0) {
		std::cout << "score: " << icp.getFitnessScore() << std::endl;
		float m1[3];
		m1[0] = 0;
		m1[1] = 0;
		m1[2] = 0;
		for (auto p : final->points) {
			m1[0] += p.x;
			m1[1] += p.y;
			m1[2] += p.z;
		}
		m1[0] /= final->points.size();
		m1[1] /= final->points.size();
		m1[2] /= final->points.size();

		std::cout << "aligned: " << m1[0] << "|" << m1[1] << "|" << m1[2]
				<< std::endl;
		pcl::visualization::PCLVisualizer visu("Final Alignment");
		visu.addPointCloud(initial, ColorHandlerT(initial, 0.0, 255.0, 0.0),
				"scene");
		visu.addPointCloud(target, ColorHandlerT(target, 255.0, 0.0, 0),
				"object");
		visu.addPointCloud(final, ColorHandlerT(final, 0.0, 0.0, 255.0),
				"object_aligned");
		visu.spin();
	}
	return icp.getFinalTransformation();
}
/**
 * Calculate the transformation between two point clouds using
 * first a simple feature matching strategy and then icp
 *
 * @param input the initial point cloud
 * @param target the target point cloud
 * @param debug flag to specify the debug level
 *         - 0 no Debug log
 *         - 1 showing to clouds in viewer
 *         - >1 showing pcl debuglog
 *
 * @return The transformation between the point clouds
 *
 */
const static Eigen::Matrix4f getTransformation(
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input,
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr target, const short debug =
				0)
{
	if (debug > 1) {
		setVerbosityLevel(pcl::console::L_DEBUG);
	}

	FeatureCloudT::Ptr object_features(new FeatureCloudT);
	FeatureCloudT::Ptr scene_features(new FeatureCloudT);
	PointCloudT::Ptr object(new PointCloudT);
	PointCloudT::Ptr object_aligned(new PointCloudT);
	PointCloudT::Ptr scene(new PointCloudT);

	//transform the pointtype
	for (auto p : input->points) {
		PointNT pn;
		pn.x = p.x;
		pn.y = p.y;
		pn.z = p.z;
		pn.normal_x = p.normal_x;
		pn.normal_y = p.normal_y;
		pn.normal_z = p.normal_z;
		pn.curvature = p.curvature;
		scene->push_back(pn);
	}

	for (auto p : target->points) {
		PointNT pn;
		pn.x = p.x;
		pn.y = p.y;
		pn.z = p.z;
		pn.normal_x = p.normal_x;
		pn.normal_y = p.normal_y;
		pn.normal_z = p.normal_z;
		pn.curvature = p.curvature;
		object->push_back(pn);
	}
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
	align.setMaxCorrespondenceDistance(1.5f /** leaf*/); // Set inlier threshold
	align.setInlierFraction(0.25f); // Set required inlier fraction
	align.align(*object_aligned);

	if (align.hasConverged()) {
		Eigen::Matrix4f transformation = align.getFinalTransformation();
		// Print results
		if (debug > 0) {

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
			pcl::console::print_info("Inliers: %i/%i\n",
					align.getInliers().size(), object->size());

			pcl::visualization::PCLVisualizer visu("Alignment befor icp");
			visu.addPointCloud(scene, ColorHandlerT(scene, 0.0, 255.0, 0.0),
					"scene");
			visu.addPointCloud(object, ColorHandlerT(object, 255.0, 0.0, 0),
					"object");
			visu.addPointCloud(object_aligned,
					ColorHandlerT(object_aligned, 0.0, 0.0, 255.0),
					"object_aligned");

			float m1[3];
			float m2[3];
			m1[0] = 0;
			m1[1] = 0;
			m1[2] = 0;
			m2[0] = 0;
			m2[1] = 0;
			m2[2] = 0;
			for (auto p : scene->points) {
				m1[0] += p.x;
				m1[1] += p.y;
				m1[2] += p.z;
			}
			m1[0] /= scene->points.size();
			m1[1] /= scene->points.size();
			m1[2] /= scene->points.size();
			for (auto p : object_aligned->points) {
				m2[0] += p.x;
				m2[1] += p.y;
				m2[2] += p.z;
			}
			m2[0] /= object_aligned->points.size();
			m2[1] /= object_aligned->points.size();
			m2[2] /= object_aligned->points.size();
			std::cout << "scene: " << m1[0] << "|" << m1[1] << "|" << m1[2]
					<< std::endl;
			std::cout << "aligned: " << m2[0] << "|" << m2[1] << "|" << m2[2]
					<< std::endl;
		}
		auto final = PointCloudT::Ptr(new PointCloudT);
		auto trafo = runICP(object_aligned, scene, debug, final);
		if (debug > 0) {
			// Show alignment
			auto visu = pcl::visualization::PCLVisualizer("Alignment All");
			visu.addPointCloud(scene, ColorHandlerT(scene, 0.0, 255.0, 0.0),
					"scene");
			visu.addPointCloud(object, ColorHandlerT(object, 255.0, 0.0, 0),
					"object");
			visu.addPointCloud(final, ColorHandlerT(final, 0.0, 0.0, 255.0),
					"object_aligned");
			visu.spin();
		}
		return transformation*trafo;
	}
	pcl::console::print_error("Alignment failed!\n");
	return Eigen::Matrix4f();
}
};
}

#endif /* IPC_H_ */
