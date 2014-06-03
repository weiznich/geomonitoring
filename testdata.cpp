/*
 * testdata.cpp
 *
 *  Created on: 05.05.2014
 *      Author: Georg Semmler
 */

#include <iostream>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/random.hpp>
#include <cmath>

#include "Parser/PlyParser.h"
#include "testdata/TestData.h"
#include "IPC/IPC.h"

void copyCloud(pcl::PointCloud<pcl::PointXYZRGBNormal> cloud,
		pcl::PointCloud<pcl::PointXYZRGBNormal>* input)
{
	for (auto p : cloud) {
		input->push_back(p);
	}
}

int main(int argc, char** argv)
{
	if (argc != 2) {
		std::cerr << "No filename passed" << std::endl;
		return 0;
	}
	//int passed = 0;
	//for (int i = 0; i < 100; i++) {
	auto cloud = Parser::PlyParser::parse(std::string(argv[1]));
	pcl::PointCloud<pcl::PointXYZRGBNormal>* input =
				new pcl::PointCloud<pcl::PointXYZRGBNormal>();
	copyCloud(cloud, input);

	pcl::PointCloud<pcl::PointXYZRGBNormal>* target =
			new pcl::PointCloud<pcl::PointXYZRGBNormal>();
	Eigen::Matrix4f trafo;
	TestData::TestData::getTrafo(cloud, trafo, target);
	Eigen::Matrix4f trafo_calc;
	pcl::PointCloud<pcl::PointXYZ>* output =
			new pcl::PointCloud<pcl::PointXYZ>();
	std::cout<<"target points: "<<target->size()<<std::endl;
//Calculation::pairAlign(pcl::PointCloud<pcl::PointXYZ>::Ptr(input),
	//		pcl::PointCloud<pcl::PointXYZ>::Ptr(target),
	//		pcl::PointCloud<pcl::PointXYZ>::Ptr(output), trafo_calc, false);
	Calculation::test(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(input),
			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(target));
	//if ( Calculation::runIPC(input, target, trafo_calc))
	//	++passed;
	//}
	std::cout << trafo_calc << std::endl;
	//std::cout << "passed " << passed << " from 100" << std::endl;

}

