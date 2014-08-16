/*
 * PlyParser.cpp
 *
 *  Created on: 28.04.2014
 *      Author: Georg Semmler
 */

#include "PlyParser.h"

#include <cstdio>//EOF
#include <fstream>

#include <algorithm>
#include <boost/algorithm/string.hpp>

namespace Parser
{

std::istream& PlyParser::safeGetline(std::istream& is, std::string& t)
{
	t.clear();

	// The characters in the stream are read one-by-one using a std::streambuf.
	// That is faster than reading them one-by-one using the std::istream.
	// Code that uses streambuf this way must be guarded by a sentry object.
	// The sentry object performs various tasks,
	// such as thread synchronization and updating the stream state.

	std::istream::sentry se(is, true);
	std::streambuf* sb = is.rdbuf();

	for (;;) {
		int c = sb->sbumpc();
		switch (c) {
		case '\r':
			c = sb->sgetc();
			if (c == '\n') {
				sb->sbumpc();
			}
			return is;
		case EOF:
		case '\0':
			is.setstate(std::istream::eofbit);
		case '\n':
			return is;
		default:
			t += (char) c;
		}
	}
}
bool PlyParser::checkValidFile(std::vector<std::string>& lines,
		size_t& vertexCount)
{
	bool isValidFile = true;
	size_t counter = 0;

	for (counter = 0; counter < 13 && counter < lines.size(); counter++) {
		std::string line = lines[counter];
		switch (counter) {
		case 0:
			isValidFile = isValidFile && (line == "ply");
			break;
		case 1:
			isValidFile = isValidFile && (line == "format ascii 1.0");
			break;
		case 2:
			isValidFile = isValidFile
					&& (line.compare(0, 14, "element vertex") == 0);
			vertexCount = std::stoi(line.substr(14, line.length() - 14));
			break;
		case 3:
			isValidFile = isValidFile && (line == "property float x");
			break;
		case 4:
			isValidFile = isValidFile && (line == "property float y");
			break;
		case 5:
			isValidFile = isValidFile && (line == "property float z");
			break;
		case 6:
			isValidFile = isValidFile && (line == "property float nx");
			break;
		case 7:
			isValidFile = isValidFile && (line == "property float ny");
			break;
		case 8:
			isValidFile = isValidFile && (line == "property float nz");
			break;
		case 9:
			isValidFile = isValidFile && (line == "property uchar red");
			break;
		case 10:
			isValidFile = isValidFile && (line == "property uchar green");
			break;
		case 11:
			isValidFile = isValidFile && (line == "property uchar blue");
			break;
		case 12:
			isValidFile == isValidFile && (line == "end_header");
			break;
		default:
			isValidFile = false;
		}
		if (!isValidFile) {
			std::cerr << "condition " << counter << " failed" << std::endl;
			break;
		}
	}
	return isValidFile && counter == 13;
}

void PlyParser::readCoordinates(pcl::PointXYZRGBNormal& p,
		const std::vector<std::string>& parts)
{
	//read coordinates
	p.x = std::stof(parts[0]);
	p.y = std::stof(parts[1]);
	p.z = std::stof(parts[2]);
}

void PlyParser::readNormal(pcl::PointXYZRGBNormal& p,
		const std::vector<std::string>& parts)
{
	p.normal_x = std::stof(parts[3]);
	p.normal_y = std::stof(parts[4]);
	p.normal_z = std::stof(parts[5]);
}

void PlyParser::readColor(pcl::PointXYZRGBNormal& p,
		const std::vector<std::string>& parts)
{
	uint8_t r = std::stoi(parts[6]);
	uint8_t g = std::stoi(parts[7]);
	uint8_t b = std::stoi(parts[8]);
	uint32_t urgb = r << 16 | g << 8 | b;
	p.rgb = *(reinterpret_cast<float *>(&urgb));
}

pcl::PointCloud<pcl::PointXYZRGBNormal> PlyParser::parse(
		const std::string& filePath)
{
	using namespace pcl;
	typedef PointXYZRGBNormal Point;
	typedef PointCloud<Point> Points;
	Points points;
	points.height = 0;	//PointCloud is unstructured
	std::ifstream ply(filePath);
	if (ply.is_open()) {
		std::string line;
		std::vector<std::string> lines;
		while (safeGetline(ply, line)) {
			boost::algorithm::trim(line);
			if (!line.empty()) {
				lines.push_back(line);
			}
		}
		ply.close();
		size_t vertexCount = 0;
		bool isValidFile = checkValidFile(lines, vertexCount);
		if (isValidFile) {
			points.width = vertexCount;
			std::string line;
			if (vertexCount == lines.size() - 13) {
				for (size_t i = 13; i < lines.size(); i++) {
					std::string& line = lines[i];
					std::vector<std::string> parts;
					boost::algorithm::split(parts, line,
							boost::algorithm::is_any_of(" "),
							boost::algorithm::token_compress_on);
					if (parts.size() == 9) {
						Point p;
						for (auto& s : parts) {
							boost::algorithm::trim(s);
						}
						readCoordinates(p, parts);
						readNormal(p, parts);
						readColor(p, parts);
						points.push_back(p);
					} else {
						std::cerr << "not enough properties" << std::endl;
						std::cerr << "only " << parts.size()
								<< " properties found" << std::endl;
					}
				}
				return points;
			} else {
				std::cerr << "vertex count does not match actual line count"
						<< std::endl;
			}
		} else {
			std::cerr << filePath << "is not a falid file " << std::endl;
		}

	} else {
		std::cerr << "Could not open " << filePath << std::endl;
	}
	points.width = 0;
	return points;

}

}

