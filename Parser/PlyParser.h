/*
 * PlyParser.h
 *
 *  Created on: 28.04.2014
 *      Author: Georg Semmler
 */

#ifndef PLYPARSER_H_
#define PLYPARSER_H_

#include <iostream>
#include <istream>
#include <string>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace Parser
{

/**
 * Simple wrapper class for PYLParser
 */
class PlyParser
{
public:
	/**
	 * This function parsers a ply-file to a PCL-Pointcloud of the point type PointXYZRGBNormal
	 *
	 * @param filePath the path of the ply-file which should be parsed
	 * @return the content of the file as unstructured pcl::PointCloud
	 */
	static pcl::PointCloud<pcl::PointXYZRGBNormal> parse(
			const std::string& filePath);
private:
	static std::istream& safeGetline(std::istream& is, std::string& t);
	static bool checkValidFile(std::vector<std::string>& lines,
			size_t& vertexCount);
	static void readCoordinates(pcl::PointXYZRGBNormal& p,
			const std::vector<std::string>& parts);
	static void readNormal(pcl::PointXYZRGBNormal& p,
			const std::vector<std::string>& parts);
	static void readColor(pcl::PointXYZRGBNormal &p,
			const std::vector<std::string>& parts);
};

}
#endif /* PLYPARSER_H_ */
