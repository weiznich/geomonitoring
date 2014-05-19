/*
 * test_ipc.cpp
 *
 *  Created on: 30.04.2014
 *      Author: Georg Semmler
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/registration/icp.h>

#include <string>
#include <iostream>

#include "../Parser/PlyParser.h"
#include "IPC.h"




int main(int argc,char** argv){

	if(argc!=3){
		std::cerr<<"Invalid argument count"<<std::endl;
		std::cerr<<"Usage: "<<argv[1]<<" initial_pointcloud.ply target_pointcloud.ply"<<std::endl;
		return 0;
	}

	pcl::PointCloud<pcl::PointXYZRGBNormal> initial=Parser::PlyParser::parse(std::string(argv[1]));
	pcl::PointCloud<pcl::PointXYZRGBNormal> target=Parser::PlyParser::parse(std::string(argv[2]));
	Eigen::Matrix4f transformation;
	Calculation::runIPC(initial,target,transformation);


}

