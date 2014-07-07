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


#include "../Parser/PlyParser.h"
#include "ICP.h"

void copyCloud(pcl::PointCloud<pcl::PointXYZRGBNormal> cloud,
		pcl::PointCloud<pcl::PointXYZRGBNormal>* input)
{
	for (auto p : cloud) {
		input->push_back(p);
	}
}

int main(int argc, char** argv)
{
	if (argc != 3) {
		std::cerr << "No filename passed" << std::endl;
		return 0;
	}
	auto cloud = Parser::PlyParser::parse(std::string(argv[1]));
	pcl::PointCloud<pcl::PointXYZRGBNormal>* input =
				new pcl::PointCloud<pcl::PointXYZRGBNormal>();
	copyCloud(cloud, input);

	cloud = Parser::PlyParser::parse(std::string(argv[2]));
	pcl::PointCloud<pcl::PointXYZRGBNormal>* target =
					new pcl::PointCloud<pcl::PointXYZRGBNormal>();
	copyCloud(cloud, target);
	std::cout<<"target points: "<<target->size()<<std::endl;
	auto trafo=Calculation::getTransformation(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(input),
			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(target));
	std::cout << "Final transformation: " << std::endl << trafo
			<< std::endl;


}

