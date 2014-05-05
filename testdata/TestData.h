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

#include <cmath>
#include <iostream>

namespace TestData
{
class TestData
{
public:
	template<typename PointT>
	static void getTrafo(const pcl::PointCloud<PointT>& source,
			Eigen::Matrix4f& trafo, pcl::PointCloud<PointT>& targetCloud)
	{
		targetCloud = pcl::PointCloud<PointT>();
		for(auto p:source){
			targetCloud.push_back(p);
		}
		Eigen::Matrix3f rot;
		float alpha = randomFloat(-1, 1);
		float n[3]={0,0,0};
		float d[3]={0,0,0};
		randomUnitVector(n);
		randomUnitVector(d);
		rot << n[1] * n[1] * (1 - cos(alpha)) + (cos(alpha)), n[1] * n[2]
				* (1 - cos(alpha)) - n[3] * sin(alpha), n[1] * n[3]
				* (1 - cos(alpha)) + n[2] * sin(alpha), n[2] * n[1]
				* (1 - cos(alpha)) + n[3] * sin(alpha), n[2] * n[2]
				* (1 - cos(alpha)) + cos(alpha), n[2] * n[3] * (1 - cos(alpha))
				- n[1] * sin(alpha), n[3] * n[1] * (1 - cos(alpha))
				- n[2] * sin(alpha), n[3] * n[2] * (1 - cos(alpha))
				+ n[1] * sin(alpha), n[3] * n[3] * (1 - cos(alpha)) + cos(alpha);
		std::cout << "Rotation: " << std::endl << rot << std::endl;
		std::cout << "Translation: " << d[0] << " , " << d[1] << " , " << d[2]
				<< std::endl;
		for (auto& p : targetCloud) {
			auto vector = p.getVector3fMap();
			vector = rot * vector;
			p.x = vector[0] + d[0];
			p.y = vector[1] + d[1];
			p.z = vector[2] + d[2];
		}
		trafo = Eigen::Matrix4f();
		trafo << rot.col(0)(0), rot.col(0)(1), rot.col(0)(2), d[0], rot.col(1)(0), rot.col(
				1)(1), rot.col(1)(2), d[1], rot.col(2)(0), rot.col(2)(1), rot.col(2)(2), d[2], 0, 0, 0, 1;
		std::cout<<"Trafo: "<<std::endl<<trafo<<std::endl;
	}
private:
	static float randomFloat(float min, float max)
	{
		boost::mt19937 rng;
		boost::uniform_real<float> u(min, max);
		boost::variate_generator<boost::mt19937&, boost::uniform_real<float> > gen(
				rng, u);
		return gen();
	}

	static void randomUnitVector(float * unit)
	{
		unit[0] = randomFloat(-10, 10);
		unit[1]= randomFloat(-10, 10);
		unit[2] = randomFloat(-10, 10);
		float abs = sqrtf(unit[0] * unit[0] +unit[1] * unit[1] + unit[2] * unit[2]);
		unit[0]/=abs;
		unit[2]/=abs;
		unit[3]/=abs;
	}
};
}

#endif /* TESTDATA_H_ */
