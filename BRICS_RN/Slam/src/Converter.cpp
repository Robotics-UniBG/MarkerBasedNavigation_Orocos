/* V1.5 Andrea Luzzana
 * KDL::Frame -> geometry_msgs::Transform -> KDL::Frame   						OK
 * KDL::Twist -> geometry_msgs::Twist -> KDL::Twist       						OK
 * geometry_msgs::Transform -> geometry_msgs::Pose -> geometry_msgs::Transform  OK
 * geometry_msgs::Pose -> KDL::Frame -> geometry_msgs::Pose						OK
 */

#include "Slam/Converter.hpp"

using namespace KDL;
Converter::Converter(){
}

Converter::~Converter(){
}


void Converter::kdlToGeometryTwist(KDL::Twist &input, geometry_msgs::Twist &output){
	//Linear velocity data
	output.linear.x = input.vel.x();
	output.linear.y = input.vel.y();
	output.linear.z = input.vel.z();

	//Angular velocity data
	output.angular.x = input.rot.x();
	output.angular.y = input.rot.y();
	output.angular.z = input.rot.z();
}

void Converter::geometryToKdlTwist(geometry_msgs::Twist &input, KDL::Twist &output){
	//Linear velocity data
	output.vel.x(input.linear.x);
	output.vel.y(input.linear.y);
	output.vel.z(input.linear.z);

	//Angular velocity data
	output.rot.x(input.angular.x);
	output.rot.y(input.angular.y);
	output.rot.z(input.angular.z);
}

void Converter::kdlToTransform(KDL::Frame &input, geometry_msgs::Transform &output){
	double x, y, z, w = 0.0;
	//get the quaternion
	input.M.GetQuaternion(x, y, z, w);
	//normalize it
	normalize(x, y, z, w);
	//save it
	output.rotation.x = x;
	output.rotation.y = y;
	output.rotation.z = z;
	output.rotation.w = w;

	//Convert the translation
	output.translation.x = input.p.data[0];
	output.translation.y = input.p.data[1];
	output.translation.z = input.p.data[2];
}

void Converter::transformToKdl(geometry_msgs::Transform &input, KDL::Frame &output){
	//Convert the rotation (sets the quaternion)
	setQuaternion(output, input.rotation.x, input.rotation.y, input.rotation.z, input.rotation.w);

	//Convert the translation
	output.p.data[0] = input.translation.x;
	output.p.data[1] = input.translation.y;
	output.p.data[2] = input.translation.z;
}


void Converter::setQuaternion(KDL::Frame &fr, double x, double y, double z, double w){
	//normalize the vector
	normalize(x, y, z, w);
	//Translate the quaternion into an orthogonal matrix
	fr.M.data[0] = 1.0 - 2.0*y*y - 2.0*z*z;
	fr.M.data[1] = 2.0*x*y - 2.0*z*w;
	fr.M.data[2] = 2.0*x*z+2.0*y*w;
	fr.M.data[3] = 2.0*x*y + 2.0*z*w;
	fr.M.data[4] = 1.0 - 2.0*x*x - 2.0*z*z;
	fr.M.data[5] = 2.0*y*z - 2.0*x*w;
	fr.M.data[6] = 2.0*x*z - 2.0*y*w;
	fr.M.data[7] = 2.0*y*z + 2.0*x*w;
	fr.M.data[8] = 1.0 - 2.0*x*x - 2.0*y*y;

}

void Converter::normalize(double &x, double &y, double &z, double &w){
	//Compute the norm of the vector
	double norm = sqrt(x*x + y*y + z*z + w*w);
	//Normalize the vector
	x/=norm;
	y/=norm;
	z/=norm;
	w/=norm;
}


void Converter::poseToTransform(geometry_msgs::Pose &pose, geometry_msgs::Transform &transform){
	transform.rotation.x = pose.orientation.x;
	transform.rotation.y = pose.orientation.y;
	transform.rotation.z = pose.orientation.z;
	transform.rotation.w = pose.orientation.w;

	transform.translation.x = pose.position.x;
	transform.translation.y = pose.position.y;
	transform.translation.z = pose.position.z;
}

void Converter::transformToPose(geometry_msgs::Transform &transform, geometry_msgs::Pose &pose){
	pose.orientation.x = transform.rotation.x;
	pose.orientation.y = transform.rotation.y;
	pose.orientation.z = transform.rotation.z;
	pose.orientation.w = transform.rotation.w;

	pose.position.x = transform.translation.x;
	pose.position.y = transform.translation.y;
	pose.position.z = transform.translation.z;
}


void Converter::poseTokdl(geometry_msgs::Pose &pose, KDL::Frame &frame){
	//convert to transform...
	geometry_msgs::Transform tr_t;
	poseToTransform(pose, tr_t);
	//... and then to pose
	transformToKdl(tr_t, frame);
}

void Converter::kdlToPose(KDL::Frame &frame, geometry_msgs::Pose &pose){
	//convert to transform...
	geometry_msgs::Transform tr_t;
	kdlToTransform(frame, tr_t);
	//...and then to pose
	transformToPose(tr_t, pose);

}

void Converter::testConversionFrame(){
	//FIRST
	double qx = 0.18;
	double qy = 0.36;
	double qz = 0.55;
	double qw = 0.73;
	double x = 1;
	double y = 2;
	double z = 3;

	RTT::log(RTT::Info)<<"**** TEST ****"<<RTT::endlog();
	RTT::log(RTT::Info)<<"**** FRAME => Transform ****"<<RTT::endlog();
	RTT::log(RTT::Info)<<"Reference: [x y z]"<<" = "<<x<<" "<<y<<" "<<z<<" "<<RTT::endlog();
	RTT::log(RTT::Info)<<"Reference: [qx qy qz qw]"<<" = "<<qx<<" "<<qy<<" "<<qz<<" "<<qw<<" "<<RTT::endlog();
	//Create a frame and a transform
	geometry_msgs::Transform tr1;
	KDL::Frame fr1;
	//Set the Frame
	fr1 = Frame(KDL::Rotation::Quaternion(qx, qy, qz, qw),//MUST be normalized values!
			KDL::Vector(x, y, z));
	//	setQuaternion(fr1, qx, qy, qz, qw);  //CAN be normalized!
	//Convert to transform
	kdlToTransform(fr1, tr1);
	//print frame and transform
	printFrame(fr1);
	printTransform(tr1);

	//SECOND
	RTT::log(RTT::Info)<<"**** Transform => FRAME ****"<<RTT::endlog();
	//Create a frame and a transform
	geometry_msgs::Transform tr2;
	KDL::Frame fr2;
	//Set the Transform
	tr2.rotation.x = qx;
	tr2.rotation.y = qy;
	tr2.rotation.z = qz;
	tr2.rotation.w = qw;
	tr2.translation.x = x;
	tr2.translation.y = y;
	tr2.translation.z = z;
	//Convert to frame
	transformToKdl(tr2, fr2);
	//print transform and frame
	printTransform(tr2);
	printFrame(fr2);
}

void Converter::testConversionTwist(){
	//FIRST
	double x_l_dot = 0.123;
	double y_l_dot = 0.456;
	double z_l_dot = 3.0;
	double x_r_dot = 2.0;
	double y_r_dot = 5.321;
	double z_r_dot = 6.987;

	RTT::log(RTT::Info)<<"**** TEST ****"<<RTT::endlog();
	RTT::log(RTT::Info)<<"**** KDL::Twist => geometry_msg::Twist ****"<<RTT::endlog();
	RTT::log(RTT::Info)<<"Reference: [Xdot_l Ydot_l Zdot_l]"<<" = "<<x_l_dot<<" "<<y_l_dot<<" "<<z_l_dot<<" "<<RTT::endlog();
	RTT::log(RTT::Info)<<"Reference: [Xdot_r Ydot_r Zdot_r]"<<" = "<<x_r_dot<<" "<<y_r_dot<<" "<<z_r_dot<<" "<<RTT::endlog();
	//Create a couple of twists
	geometry_msgs::Twist gm_tw1;
	KDL::Twist kdl_tw1;
	//Set the kdl twist
	kdl_tw1.rot.data[0] = x_r_dot;
	kdl_tw1.rot.data[1] = y_r_dot;
	kdl_tw1.rot.data[2] = z_r_dot;
	kdl_tw1.vel.data[0] = x_l_dot;
	kdl_tw1.vel.data[1] = y_l_dot;
	kdl_tw1.vel.data[2] = z_l_dot;
	//	setQuaternion(fr1, qx, qy, qz, qw);  //CAN be normalized!
	//Convert to transform
	kdlToGeometryTwist(kdl_tw1, gm_tw1);
	//print twists
	printTwist(kdl_tw1);
	printTwist(gm_tw1);

	//SECOND
	RTT::log(RTT::Info)<<"**** geometry_msg::Twist => KDL::Twist ****"<<RTT::endlog();
	//Create a couple of twists
	geometry_msgs::Twist gm_tw2;
	KDL::Twist kdl_tw2;
	//Set the gmtwist
	gm_tw2.angular.x = x_r_dot;
	gm_tw2.angular.y = y_r_dot;
	gm_tw2.angular.z = z_r_dot;
	gm_tw2.linear.x = x_l_dot;
	gm_tw2.linear.y = y_l_dot;
	gm_tw2.linear.z = z_l_dot;
	//Convert twists
	geometryToKdlTwist(gm_tw2, kdl_tw2);
	//print transform and frame
	printTwist(gm_tw2);
	printTwist(kdl_tw2);
}

void Converter::printFrame(KDL::Frame frame){
	RTT::log(RTT::Info)<<"This is a KDL::Frame (no normalizing when printing)"<<RTT::endlog();
	RTT::log(RTT::Info)<<"X:"<<frame.p.data[0]<<" Y:"<<frame.p.data[1]<<" Z:"<<frame.p.data[2]<<RTT::endlog();
	double qx, qy, qz, qw;
	frame.M.GetQuaternion(qx, qy, qz, qw);
	RTT::log(RTT::Info)<<"QX: "<<qx<<" QY: "<<qy<<" QZ:"<<qz<<" QW:"<<qw<<RTT::endlog();
}

void Converter::printTransform(geometry_msgs::Transform tran){
	RTT::log(RTT::Info)<<"This is a geometry_msgs::Transform"<<RTT::endlog();
	RTT::log(RTT::Info)<<"X:"<<tran.translation.x<<" Y:"<<tran.translation.y<<" Z:"<<tran.translation.z<<RTT::endlog();
	RTT::log(RTT::Info)<<"QX: "<<tran.rotation.x<<" QY: "<<tran.rotation.y<<" QZ:"<<tran.rotation.z<<" QW:"<<tran.rotation.w<<RTT::endlog();
}

void printPose(geometry_msgs::Pose &p){
	RTT::log(RTT::Info)<<"This is a geometry_msgs::Pose"<<RTT::endlog();
	RTT::log(RTT::Info)<<"X:"<<p.position.x<<" Y:"<<p.position.y<<" Z:"<<p.position.z<<RTT::endlog();
	RTT::log(RTT::Info)<<"QX: "<<p.orientation.x<<" QY: "<<p.orientation.y<<" QZ:"<<p.orientation.z<<" QW:"<<p.orientation.w<<RTT::endlog();
}

void Converter::printTwist(geometry_msgs::Twist tw){
	RTT::log(RTT::Info)<<"This is a geometry_msgs::Twist"<<RTT::endlog();
	RTT::log(RTT::Info)<<"Xdot_l:"<<tw.linear.x<<" Ydot_l:"<<tw.linear.y<<" Zdot_l:"<<tw.linear.z<<RTT::endlog();
	RTT::log(RTT::Info)<<"Xdot_r:"<<tw.angular.x<<" Ydot_r:"<<tw.angular.y<<" Zdot_r:"<<tw.angular.z<<RTT::endlog();
}

void Converter::printTwist(KDL::Twist tw){
	RTT::log(RTT::Info)<<"This is a KDL::Twist"<<RTT::endlog();
	RTT::log(RTT::Info)<<"Xdot_l:"<<tw.vel.data[0]<<" Ydot_l:"<<tw.vel.data[1]<<" Zdot_l:"<<tw.vel.data[2]<<RTT::endlog();
	RTT::log(RTT::Info)<<"Xdot_r:"<<tw.rot.data[0]<<" Ydot_r:"<<tw.rot.data[1]<<" Zdot_r:"<<tw.rot.data[2]<<RTT::endlog();
}



