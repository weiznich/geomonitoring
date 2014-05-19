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

#include <string>
#include <iostream>

#include <cmath>

namespace Calculation{
void runIPC(pcl::PointCloud<pcl::PointXYZRGBNormal> initial, pcl::PointCloud<pcl::PointXYZRGBNormal> target,Eigen::Matrix4f& transformation,bool debug=false){
	pcl::IterativeClosestPoint<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icp;
	pcl::PointCloud<pcl::PointXYZRGBNormal> Final;
	if(debug)
		setVerbosityLevel(pcl::console::L_DEBUG);

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr iPtr(&initial);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr fPtr(&target);
	//icp.setMaxCorrespondenceDistance (0.1);
	int counter=0;
	float score=0;
	do{
		icp.setMaximumIterations(pow10(counter++));
		icp.setInputSource(iPtr);
		icp.setInputTarget(fPtr);
		icp.align(Final);
		std::cout <<" score: " <<  icp.getFitnessScore() << std::endl;
		/*if(abs(score-icp.getFitnessScore())<pow10(-1*(counter+2))&&score!=0){
			std::cout<<"does not converge"<<std::endl;
			break;
		}*/
		score=icp.getFitnessScore();
	}while(counter<4&&icp.getFitnessScore()>0.00000001);
	transformation = icp.getFinalTransformation();
	if(icp.getFitnessScore()<0.00000001){
		std::cout<<"converge"<<std::endl;
	}
    std::cout<<"transform: "<<std::endl<<transformation<<std::endl;
}

}



#endif /* IPC_H_ */
