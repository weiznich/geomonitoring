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

/*
 * This function copy PCL pointclouds from input to output
 *
 * @param input, the cloud which should be copied
 *
 * @param output, the copied cloud, output
 */
void copyCloud(const pcl::PointCloud<pcl::PointXYZRGBNormal>* const input,
		pcl::PointCloud<pcl::PointXYZRGBNormal>* output)
{
	for (auto p : input->points) {
		output->push_back(p);
	}
}


//mainfunction
int main(int argc, char** argv)
{
	//check arguments
	if (argc <3 || argc>4) {
		std::cout<<argc<<std::endl;
		std::cerr << "call "<<argv[0]<<" <path to inputcloud> <path to targetcloud> [<outputpath>]" << std::endl;
		return 0;
	}
	boost::optional<std::string> output_path=boost::none;
	if(argc==4){
		output_path=std::string(argv[3]);
	}
	//read inputcloud
	auto cloud = Parser::PlyParser::parse(std::string(argv[1]));
	pcl::PointCloud<pcl::PointXYZRGBNormal>* input =
				new pcl::PointCloud<pcl::PointXYZRGBNormal>();
	copyCloud(&cloud, input);

	//read target cloud
	cloud = Parser::PlyParser::parse(std::string(argv[2]));
	pcl::PointCloud<pcl::PointXYZRGBNormal>* target =
					new pcl::PointCloud<pcl::PointXYZRGBNormal>();
	copyCloud(&cloud, target);

	//calculate transformation
	auto trafo=Calculation::ICP::getTransformation(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(input),
			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(target),output_path);
	std::cout << "Final transformation: " << std::endl << trafo
			<< std::endl;


}

