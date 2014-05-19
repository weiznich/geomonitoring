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

int main(int argc, char** argv){
	if(argc!=2){
		std::cerr<<"No filename passed"<<std::endl;
		return 0;
	}
	auto cloud=Parser::PlyParser::parse(std::string(argv[1]));
	pcl::PointCloud<pcl::PointXYZRGBNormal> target;
	Eigen::Matrix4f trafo;
	TestData::TestData::getTrafo(cloud,trafo,target);
	Eigen::Matrix4f trafo_calc;
	Calculation::runIPC(cloud,target,trafo_calc);

}




