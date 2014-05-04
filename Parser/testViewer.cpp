/*
 * testViewer.cpp
 *
 *  Created on: 28.04.2014
 *      Author: Georg Semmler
 */
#include <string>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/impl/point_types.hpp>
#include <boost/make_shared.hpp>
#include "PlyParser.h"

int main(int argc, char** argv)
{
	pcl::visualization::CloudViewer viewer("Clouds");
	for (int i = 1; i < argc; i++) {
		std::string path(argv[i]);
		pcl::PointCloud<pcl::PointXYZRGBNormal> cloud=Parser::PlyParser::parse(path);
		pcl::PointCloud<pcl::PointXYZRGB>copy;
		for(const auto& p:cloud){
			pcl::PointXYZRGB pt;
			pt.x=p.x;
			pt.y=p.y;
			pt.z=p.z;

			pt.rgb=p.rgb;
			copy.push_back(pt);
		}
		copy.width=cloud.width;
		viewer.showCloud(copy.makeShared(),std::string(argv[i]));

	}
	while (!viewer.wasStopped()) {
		//wait to close
	}
	return 1;
}


