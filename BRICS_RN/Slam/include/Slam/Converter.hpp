/* V1.5 Andrea Luzzana
 * KDL::Frame -> geometry_msgs::Transform -> KDL::Frame   						OK
 * KDL::Twist -> geometry_msgs::Twist -> KDL::Twist       						OK
 * geometry_msgs::Transform -> geometry_msgs::Pose -> geometry_msgs::Transform  OK
 * geometry_msgs::Pose -> KDL::Frame -> geometry_msgs::Pose						OK
 */



#ifndef CONVERTER_H
#define CONVERTER_H

#include <rtt/Logger.hpp>
#include <rtt/Property.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Service.hpp>
#include <rtt/Component.hpp>

#include <kdl/frames_io.hpp>>
#include <geometry_msgs/typekit/Types.h>

using namespace std;

class Converter{

public:
	Converter(void);
	~Converter(void);
	void kdlToGeometryTwist(KDL::Twist &input, geometry_msgs::Twist &output);
	void geometryToKdlTwist(geometry_msgs::Twist &input, KDL::Twist &output);
	void kdlToTransform(KDL::Frame &input, geometry_msgs::Transform &output);
	void transformToKdl(geometry_msgs::Transform &input, KDL::Frame &output);
	void poseToTransform(geometry_msgs::Pose &pose, geometry_msgs::Transform &transform);
	void transformToPose(geometry_msgs::Transform &transform, geometry_msgs::Pose &pose);
	void poseTokdl(geometry_msgs::Pose &pose, KDL::Frame &frame);
	void kdlToPose(KDL::Frame &frame, geometry_msgs::Pose &pose);

	void testConversionFrame();
	void testConversionTwist();
	void printFrame(KDL::Frame frame);
	void printTransform(geometry_msgs::Transform tran);
	void printPose(geometry_msgs::Pose p);
	void printTwist(geometry_msgs::Twist tw);
	void printTwist(KDL::Twist tw);

private:
	void normalize(double &x, double &y, double &z, double &w);
	void setQuaternion(KDL::Frame &fr, double x, double y, double z, double w);

};

#endif
