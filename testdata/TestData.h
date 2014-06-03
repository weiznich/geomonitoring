/*
 * TestData.h
 *
 *  Created on: 05.05.2014
 *      Author: Georg Semmler
 */

#ifndef TESTDATA_H_
#define TESTDATA_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <boost/random.hpp>
#include <boost/random/random_device.hpp>

#include <cmath>
#include <iostream>

namespace TestData
{
class TestData
{
public:
	template<typename PointT>
	static void getTrafo(const pcl::PointCloud<PointT>& source,
			Eigen::Matrix4f& trafo, pcl::PointCloud<PointT>* targetCloud)
	{
		delete targetCloud;
		targetCloud = new pcl::PointCloud<PointT>();
		for(auto p:source){
			targetCloud->push_back(p);
		}
		Eigen::Matrix4f rot;
		float alpha = randomFloat(-0.1,0.1);
		std::cout<<"alpha "<<alpha<<std::endl;
		float n[3]={0,0,0};
		float d[3]={0,0,0};
		randomUnitVector(n);
		randomUnitVector(d);
		rot << n[1] * n[1] * (1 - cos(alpha)) + (cos(alpha)), n[1] * n[2]
				* (1 - cos(alpha)) - n[3] * sin(alpha), n[1] * n[3]
				* (1 - cos(alpha)) + n[2] * sin(alpha),d[0], n[2] * n[1]
				* (1 - cos(alpha)) + n[3] * sin(alpha), n[2] * n[2]
				* (1 - cos(alpha)) + cos(alpha), n[2] * n[3] * (1 - cos(alpha))
				- n[1] * sin(alpha),d[1], n[3] * n[1] * (1 - cos(alpha))
				- n[2] * sin(alpha), n[3] * n[2] * (1 - cos(alpha))
				+ n[1] * sin(alpha), n[3] * n[3] * (1 - cos(alpha)) + cos(alpha),d[2],0,0,0,1;
		std::cout << "Rotation: " << std::endl << rot << std::endl;
		std::cout << "Translation: " << d[0] << " , " << d[1] << " , " << d[2]
				<< std::endl;
		for (auto& p : targetCloud->points) {
			Eigen::Vector4f norm;
			norm<<p.x,p.y,p.z,1;
			norm=rot*norm;
			p.x = norm[0];
			p.y = norm[1];
			p.z = norm[2];
		}

		std::cout<<"Trafo: "<<std::endl<<rot<<std::endl;
	}
private:
	static float randomFloat(float min, float max)
	{
		boost::mt19937 rng;

		boost::uniform_real<float> u(min, max);
		boost::variate_generator<boost::mt19937&, boost::uniform_real<float> > gen(
				rng, u);
		rng.seed(boost::random::random_device()());
		return gen();
	}

	static void randomUnitVector(float * unit)
	{
		unit[0] = randomFloat(-10, 10);
		unit[1]= randomFloat(-10, 10);
		unit[2] = randomFloat(-10, 10);
		std::cout<<unit[0]<<","<<unit[1]<<","<<unit[2]<<std::endl;
		float abs = sqrtf(unit[0] * unit[0] +unit[1] * unit[1] + unit[2] * unit[2]);
		std::cout<<"abs: "<<abs<<std::endl;
		unit[0]/=abs;
		unit[1]/=abs;
		unit[2]/=abs;
		abs = sqrtf(unit[0] * unit[0] +unit[1] * unit[1] + unit[2] * unit[2]);
		std::cout<<"abs "<<abs<<std::endl;
	}
};
}

#endif /* TESTDATA_H_ */
