/*
 * thresholdFilter.cpp
 *
 *  Created on: Jun 8, 2012
 *      Author: andrea
 */

#include "Slam/thresholdFilter.hpp"

//Constructor, initializes all thresholds to zero [0] (No filtering)
thresholdFilter::thresholdFilter() {
	this->xTh = 0;
	this->yTh = 0;
	this->zTh = 0;
	this->qxTh = 0;
	this->qyTh = 0;
	this->qzTh = 0;
	this->qwTh = 0;
}

thresholdFilter::~thresholdFilter() {

}

//Set all thresholds [m] and [rad]
void thresholdFilter::setThresholds(	double xTh,	double yTh, double zTh,	double qxTh, double qyTh, double qzTh, double qwTh){
	this->xTh = xTh;
	this->yTh = yTh;
	this->zTh = zTh;
	this->qxTh = qxTh;
	this->qyTh = qyTh;
	this->qzTh = qzTh;
	this->qwTh = qwTh;
}

//Set separate thresholds for the translation and the rotation [m] and [rad]
void thresholdFilter::setThresholds(double traslTh,	double rotoTh){
	this->xTh = traslTh;
	this->yTh = traslTh;
	this->zTh = traslTh;
	this->qxTh = rotoTh;
	this->qyTh = rotoTh;
	this->qzTh = rotoTh;
	this->qwTh = rotoTh;
}

//returns true if the specified transforms are different, false if they are equal.
//The comparison is made by checking the 6-DOF distance between transforms with the specified thresholds
//see setThresholds methods.
bool thresholdFilter::areDifferent(geometry_msgs::Transform& oldTr, geometry_msgs::Transform& newTr){
	bool areDifferent = false;
	//Check translations
	if(fabs(oldTr.translation.x - newTr.translation.x) > this->xTh)
		areDifferent = true;
	else if(fabs(oldTr.translation.y - newTr.translation.y) > this->yTh)
		areDifferent = true;
	else if(fabs(oldTr.translation.z - newTr.translation.z) > this->zTh)
		areDifferent = true;

	//Check rotations
	else if(fabs(diffAngles(oldTr.rotation.x, newTr.rotation.x)) > this->qxTh)
		areDifferent = true;
	else if(fabs(diffAngles(oldTr.rotation.y, newTr.rotation.y)) > this->qyTh)
		areDifferent = true;
	else if(fabs(diffAngles(oldTr.rotation.z, newTr.rotation.z)) > this->qzTh)
		areDifferent = true;
	else if(fabs(diffAngles(oldTr.rotation.w, newTr.rotation.w)) > this->qwTh)
		areDifferent = true;

	return areDifferent;
}

//Shortest distance between two angles [rad]
double thresholdFilter::diffAngles(double goalAngle, double actualAngle) {
	return atan2(sin(goalAngle-actualAngle), cos(goalAngle-actualAngle));
}

