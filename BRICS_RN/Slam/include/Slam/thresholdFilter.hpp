/*
 * thresholdFilter.hpp
 *
 *  Created on: Jun 8, 2012
 *      Author: andrea
 */

#ifndef THRESHOLDFILTER_HPP_
#define THRESHOLDFILTER_HPP_
#include "geometry_msgs/Transform.h"

class thresholdFilter {
public:
	thresholdFilter();
	~thresholdFilter();
	void setThresholds(	double xTh,	double yTh, double zTh,	double qxTh, double qyTh, double qzTh, double QwTh);
	void setThresholds(	double traslTh,	double rotoTh);
	bool areDifferent(geometry_msgs::Transform& oldTr, geometry_msgs::Transform& newTr);

private:
	double xTh;
	double yTh;
	double zTh;
	double qxTh;
	double qyTh;
	double qzTh;
	double qwTh;
	double diffAngles (double goalAngle, double actualAngle);

};

#endif /* THRESHOLDFILTER_HPP_ */
