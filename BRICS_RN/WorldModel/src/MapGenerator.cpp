/********************************************************************************
 *
 * WorldModel
 *
 * Copyright (c) 2012
 * All rights reserved.
 *
 * Luca Gherardi and Alexey Zakharov
 * University of Bergamo
 * Dept. of Information Technology and Mathematics
 *
 * -------------------------------------------------------------------------------
 *
 * File: WorldModel.cpp
 * Created: June 07, 2012
 *
 * Author: <A HREF="mailto:luca.gherardi@unibg.it">Luca Gherardi</A>
 *
 * Supervised by: <A HREF="mailto:brugali@unibg.it">Davide Brugali</A>
 *
 * -------------------------------------------------------------------------------
 *
 * This sofware is published under a dual-license: GNU Lesser General Public
 * License LGPL 2.1 and BSD license. The dual-license implies that users of this
 * code may choose which terms they prefer.
 *
 * -------------------------------------------------------------------------------
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  - Neither the name of the University of Bergamo nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 2.1 of the
 * License, or (at your option) any later version or the BSD license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL and the BSD license for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL and BSD license along with this program.
 *
 *******************************************************************************/


#include <cstdio>
#include <libgen.h>
#include <fstream>


#include <SDL/SDL_image.h>
#include <yaml-cpp/yaml.h>

#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>

using namespace std;

int main( int argc, const char* argv[] ){

	cout << "Param number" << argc << endl;

	if(argc < 2){

		cout << "missing parameter" << endl;
		exit(-1);

	}

	string mapDescriptionPath = argv[1];

	int mapWidth;
	int mapHeight;
	int boardX;
	int boardY;
	float origin[3];
	float resolution;
	float freeThreshold;
	float occupiedThreshold;
	string mapFile;

	std::ifstream mapDescritptorFile(mapDescriptionPath.c_str());
	if (mapDescritptorFile.fail()) {
		cout << "Map_server could not open " << mapDescriptionPath.c_str() << endl;
		return false;
	}
	YAML::Parser parser(mapDescritptorFile);
	YAML::Node doc;
	parser.GetNextDocument(doc);

	try {
		doc["fileName"] >> mapFile;
	} catch (YAML::InvalidScalar&) {
		cout << "The map does not contain a fileName tag or it is invalid." << endl;
		return false;
	}

	try {
		doc["resolution"] >> resolution;
	} catch (YAML::InvalidScalar&) {
		cout << "The map does not contain a resolution tag or it is invalid." << endl;
		return false;
	}

	try {
		doc["width"] >> mapWidth;
	} catch (YAML::InvalidScalar&) {
		cout << "The map does not contain a width tag or it is invalid." << endl;
		return false;
	}

	try {
		doc["height"] >> mapHeight;
	} catch (YAML::InvalidScalar&) {
		cout << "The map does not contain a height tag or it is invalid." << endl;
		return false;
	}

	try {
		doc["boardX"] >> boardX;
	} catch (YAML::InvalidScalar&) {
		cout << "The map does not contain a boardX tag or it is invalid." << endl;
		return false;
	}

	try {
		doc["boardY"] >> boardY;
	} catch (YAML::InvalidScalar&) {
		cout << "The map does not contain a boardY tag or it is invalid." << endl;
		return false;
	}
	if(doc.FindValue("freeThreshold")){
		try {
			doc["origin"][0] >> origin[0];
			doc["origin"][1] >> origin[1];
			doc["origin"][2] >> origin[2];
		} catch (YAML::InvalidScalar&) {
			cout << "The map origin tag is invalid." << endl;
			return false;
		}
	}else{
		origin[0] = 0.0;
		origin[1] = 0.0;
		origin[2] = 0.0;
	}

	if(doc.FindValue("freeThreshold")){
		try {
			doc["freeThreshold"] >> freeThreshold;
		} catch (YAML::InvalidScalar&) {
			cout << "The map freeThreshold tag is invalid." << endl;
			return false;
		}
	}else{
		freeThreshold = 0.196;
	}

	if(doc.FindValue("occupiedThreshold")){
		try {
			doc["occupiedThreshold"] >> freeThreshold;
		} catch (YAML::InvalidScalar&) {
			cout << "The map occupiedThreshold tag is invalid." << endl;
			return false;
		}
	}else{
		occupiedThreshold = 0.65;
	}

	///////////////////////
	// CREATE ARENA MAP
	//////////////////////

	nav_msgs::OccupancyGrid map;

	map.info.resolution = resolution;
	map.info.width = mapWidth;
	map.info.height = mapHeight;
	map.data.resize(mapWidth * mapHeight);
	map.info.origin.position.x = origin[0];
	map.info.origin.position.y = origin[1];
	map.info.origin.position.z = 0.0;
	tf::Quaternion q;
	q.setRPY(0,0, origin[2]);
	map.info.origin.orientation.x = q.x();
	map.info.origin.orientation.y = q.y();
	map.info.origin.orientation.z = q.z();
	map.info.origin.orientation.w = q.w();

	int data[mapHeight][mapWidth];
	for(int y = 0; y < mapHeight; y++){
		for(int x = 0; x < mapWidth; x++){
			data[y][x] = 0;
		}

	}

	int i = 0;
	while(true){

		int startX;
		int startY;
		int dimX;
		int dimY;

		stringstream segmentName;
		segmentName << "segment-" << i;

		if(!doc.FindValue(segmentName.str())){
			break;
		}

		try {
			doc[segmentName.str()][0] >> startX;
			doc[segmentName.str()][1] >> startY;
			doc[segmentName.str()][2] >> dimX;
			doc[segmentName.str()][3] >> dimY;
		} catch (YAML::InvalidScalar&) {
			cout << "Error in the key " << segmentName.str() << endl;
			exit(-1);
		}

		for( int y = startY + boardY; y < startY + dimY + boardY; y++){
			for( int x = startX + boardX; x < startX + dimX + boardX; x++){
				data[mapHeight-1-y][x] = +100;
			}
		}
		cout << "Inserted line from " << startX << "," << startY << " to " <<
				startX + dimX + boardX << "," << startY + dimY + boardY << endl;

		i++;
	}

	string debugFile = "maps/debugMap.pmg";
	FILE* outDeb = fopen(debugFile.c_str(), "w");
	if (!outDeb)
	{
		cout << "Couldn't save map file to " << debugFile.c_str() << endl;
	}else{

		fprintf(outDeb, "P5\n# CREATOR: Map_generator.cpp %.3f m/pix\n%d %d\n255\n",
				resolution, mapWidth, mapHeight);



		for(int y = 0; y < mapHeight; y++) {
			for(int x = 0; x < mapWidth; x++) {
				//unsigned int i = x + (map->info.height - y - 1) * map->info.width;
				if (data[y][x] == 0) { //occ [0,0.1)
					fputc(254, outDeb);
				} else if (data[y][x] == +100) { //occ (0.65,1]
					fputc(000, outDeb);
				} else { //occ [0.1,0.65]
					fputc(205, outDeb);
				}
			}
		}
	}

	fclose(outDeb);

	for(int y = 0; y < mapHeight; y++){
		for(int x = 0; x < mapWidth; x++){

			map.data[y * mapWidth + x] = data[y][x];

		}

	}

	std::string mapdatafile = "maps/" + mapFile + ".pgm";
	cout << "Writing map occupancy data to " << mapdatafile.c_str() << endl;
	FILE* out = fopen(mapdatafile.c_str(), "w");
	if (!out)
	{
		cout << "Couldn't save map file to " <<  mapdatafile.c_str() << endl;
		exit(-1);
	}

	fprintf(out, "P5\n# CREATOR: Map_generator.cpp %.3f m/pix\n%d %d\n255\n",
			map.info.resolution, map.info.width, map.info.height);
	for(unsigned int y = 0; y < map.info.height; y++) {
		for(unsigned int x = 0; x < map.info.width; x++) {
			unsigned int i = x + (map.info.height - y - 1) * map.info.width;

			if (map.data[i] == 0) { //occ [0,0.1)
				fputc(254, out);
			} else if (map.data[i] == +100) { //occ (0.65,1]
				fputc(000, out);
			} else { //occ [0.1,0.65]
				fputc(205, out);
			}
		}
	}

	fclose(out);

	std::string mapmetadatafile = "maps/" + mapFile + ".yaml";
	cout << "Writing map occupancy data to "<<  mapmetadatafile.c_str() << endl;
	FILE* yaml = fopen(mapmetadatafile.c_str(), "w");

	geometry_msgs::Quaternion orientation = map.info.origin.orientation;
	btMatrix3x3 mat(btQuaternion(orientation.x, orientation.y, orientation.z, orientation.w));
	double yaw = tf::getYaw(orientation);

	mapdatafile = mapFile + ".pgm";

	fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: %f\nfree_thresh: %f\n\n",
			mapdatafile.c_str(), map.info.resolution, map.info.origin.position.x, map.info.origin.position.y, yaw,
			occupiedThreshold, freeThreshold);

	fclose(yaml);







}








