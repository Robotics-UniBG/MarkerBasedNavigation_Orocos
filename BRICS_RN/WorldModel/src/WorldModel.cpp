/********************************************************************************
 *
 * WorldModel
 *
 * Copyright (c) 2012
 * All rights reserved.
 *
 * Luca Gherardi
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

#include "WorldModel/WorldModel.hpp"

#include <rtt/os/main.h>

#include <rtt/Logger.hpp>
#include <rtt/Property.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Service.hpp>
#include <rtt/Component.hpp>

#include <math.h>
#include <sys/time.h>
#include <cstdio>
#include <libgen.h>
#include <fstream>

#include <tf/tf.h>

#include <SDL/SDL_image.h>
#include "yaml-cpp/yaml.h"

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

namespace Navigation{


WorldModel::WorldModel(string const& name)
: TaskContext(name),
  occupancyGridPtrOutPort("occupancyGridPtrOutPort"),
  occupancyGridOutPort("occupancyGridOutPort"),
  mapIdInPort("mapIdInPort")
{

	this->addPort(occupancyGridPtrOutPort).doc("The occupancy grid output port");
	this->addPort(occupancyGridOutPort).doc("The occupancy grid output port");

	this->addEventPort(mapIdInPort).doc("The map id input port");

	this->addAttribute("trig",trig);
	this->addProperty("mapName",mapName);

	occupancyGridsMap.clear();

	marshalling = this->getProvider<Marshalling>("marshalling");


	trig = false;
	mapName = "testmap";


}

bool WorldModel::startHook(){

	return true;

}

bool WorldModel::configureHook(){

	return true;

}

void WorldModel::stopHook(){



}

void WorldModel::cleanupHook(){
	marshalling->writeProperties("properties/WorldModel.cpf");
}

void WorldModel::updateHook() {

	std::string mapIdInput;


	if(mapIdInPort.read(mapIdInput) == NewData || trig){

		nav_msgs::OccupancyGridConstPtr occupancyGridPtr;

		if(trig){
			trig = false;
			mapIdInput = mapName;
		}

		// check if the map was already created
		if(occupancyGridsMap.count(mapIdInput) == 0){

			cout << "Map never parsed" << endl;

			stringstream fileNameStream;
			fileNameStream << "maps/" << mapIdInput << ".yaml";

			nav_msgs::OccupancyGrid occupancyGrid;

			if(loadOccupancyGridFromFile(fileNameStream.str(), occupancyGrid)){
				occupancyGridPtr = boost::shared_ptr<nav_msgs::OccupancyGrid>(
						new nav_msgs::OccupancyGrid(occupancyGrid));
				occupancyGridsMap[mapIdInput] = occupancyGridPtr;
			}else{
				return;
			}



		}else{

			cout << "Map already parsed" << endl;
			occupancyGridPtr = occupancyGridsMap.find(mapIdInput)->second;

		}

		occupancyGridOutPort.write(*occupancyGridPtr);
		occupancyGridPtrOutPort.write(occupancyGridPtr);

	}


}

bool WorldModel::loadOccupancyGridFromFile(string mapDescriptionPath,
		nav_msgs::OccupancyGrid& occupancyGrid){

	/**
	 * Pixels with occupancy probability greater than this
	 * threshold are considered completely occupied.
	 */
	double occupancyThreshold;

	/**
	 * Pixels with occupancy probability less than this
	 * threshold are considered completely free.
	 */
	double freeThreshold;

	/**
	 * Whether the white/black free/occupied semantics should
	 * be reversed (interpretation of thresholds is unaffected)
	 */
	double negate;

	/**
	 * Resolution of the map, meters / pixel
	 */
	double resolution;

	/**
	 * The 2-D pose of the lower-left pixel in the map, as (x, y, yaw),
	 * with yaw as counterclockwise rotation (yaw=0 means no rotation).
	 */
	double origin[3];

	string imagePath;

	SDL_Surface* img;

	unsigned char* pixels;
	unsigned char* p;
	int rowstride, n_channels;
	unsigned int i,j;
	int k;
	double occ;
	int color_sum;
	double color_avg;


	std::ifstream mapDescritptorFile(mapDescriptionPath.c_str());
	if (mapDescritptorFile.fail()) {
		log(Error) << "Map_server could not open " << mapDescriptionPath.c_str() << endlog();
		return false;
	}
	YAML::Parser parser(mapDescritptorFile);
	YAML::Node doc;
	parser.GetNextDocument(doc);
	try {
		doc["resolution"] >> resolution;
	} catch (YAML::InvalidScalar) {
		log(Error) << "The map does not contain a resolution tag or it is invalid." << endlog();
		return false;
	}
	try {
		doc["negate"] >> negate;
	} catch (YAML::InvalidScalar) {
		log(Error) << "The map does not contain a negate tag or it is invalid." << endlog();
		return false;
	}
	try {
		doc["occupied_thresh"] >> occupancyThreshold;
	} catch (YAML::InvalidScalar) {
		log(Error) << "The map does not contain an occupied_thresh tag or it is invalid." << endlog();
		return false;
	}
	try {
		doc["free_thresh"] >> freeThreshold;
	} catch (YAML::InvalidScalar) {
		log(Error) << "The map does not contain a free_thresh tag or it is invalid." << endlog();
		return false;
	}
	try {
		doc["origin"][0] >> origin[0];
		doc["origin"][1] >> origin[1];
		doc["origin"][2] >> origin[2];
	} catch (YAML::InvalidScalar) {
		log(Error) << "The map does not contain an origin tag or it is invalid." << endlog();
		return false;
	}
	try {
		doc["image"] >> imagePath;
		// TODO: make this path-handling more robust
		if(imagePath.size() == 0)
		{
			log(Error) << "The image tag cannot be an empty string." << endlog();
			return false;
		}
		if(imagePath[0] != '/')
		{
			// dirname can modify what you pass it
			char* fname_copy = strdup(mapDescriptionPath.c_str());
			imagePath = std::string(dirname(fname_copy)) + '/' + imagePath;
			free(fname_copy);
		}
	} catch (YAML::InvalidScalar) {
		log(Error) << "The map does not contain an image tag or it is invalid." << endlog();
		return false;
	}
	// Load the image using SDL.  If we get NULL back, the image load failed.
	if(!(img = IMG_Load(imagePath.c_str())))
	{
		std::string errmsg = std::string("failed to open image file \"") +
				std::string(imagePath) + std::string("\"");
		//		throw std::runtime_error(errmsg);
		log(Error) << errmsg << endlog();
		return false;
	}

	// Copy the image data into the map structure
	occupancyGrid.info.width = img->w;
	occupancyGrid.info.height = img->h;
	occupancyGrid.info.resolution = resolution;
	occupancyGrid.info.origin.position.x = origin[0];
	occupancyGrid.info.origin.position.y = origin[1];
	occupancyGrid.info.origin.position.z = 0.0;
	tf::Quaternion q;
	q.setRPY(0,0, origin[2]);
	occupancyGrid.info.origin.orientation.x = q.x();
	occupancyGrid.info.origin.orientation.y = q.y();
	occupancyGrid.info.origin.orientation.z = q.z();
	occupancyGrid.info.origin.orientation.w = q.w();
	occupancyGrid.header.frame_id = "/map";

	// Allocate space to hold the data
	occupancyGrid.data.resize(occupancyGrid.info.width * occupancyGrid.info.height);

	// Get values that we'll need to iterate through the pixels
	rowstride = img->pitch;
	n_channels = img->format->BytesPerPixel;

	// Copy pixel data into the map structure
	pixels = (unsigned char*)(img->pixels);
	for(j = 0; j < occupancyGrid.info.height; j++)
	{
		for (i = 0; i < occupancyGrid.info.width; i++)
		{
			// Compute mean of RGB for this pixel
			p = pixels + j*rowstride + i*n_channels;

			color_sum = 0;
			for(k=0;k<n_channels;k++)
				color_sum += *(p + (k));
			color_avg = color_sum / (double)n_channels;

			// If negate is true, we consider blacker pixels free, and whiter
			// pixels free.  Otherwise, it's vice versa.
			if(negate)
				occ = color_avg / 255.0;
			else
				occ = (255 - color_avg) / 255.0;
			// Apply thresholds to RGB means to determine occupancy values for
			// map.  Note that we invert the graphics-ordering of the pixels to
			// produce a map with cell (0,0) in the lower-left corner.
			if(occ > occupancyThreshold)
				occupancyGrid.data[MAP_IDX(occupancyGrid.info.width,i,occupancyGrid.info.height - j - 1)] = +100;
			else if(occ < freeThreshold)
				occupancyGrid.data[MAP_IDX(occupancyGrid.info.width,i,occupancyGrid.info.height - j - 1)] = 0;
			else
				occupancyGrid.data[MAP_IDX(occupancyGrid.info.width,i,occupancyGrid.info.height - j - 1)] = -1;

		}
	}
	SDL_FreeSurface(img);

	//	nav_msgs::OccupancyGridConstPtr occupancyGridPtr = boost::shared_ptr<nav_msgs::OccupancyGrid const>(
	//			new nav_msgs::OccupancyGrid(occupancyGrid));

	return true;

}

}

ORO_CREATE_COMPONENT( Navigation::WorldModel );

