#include <cmath>
#include <cstdlib>
#include <unistd.h>
#include <iostream>
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

using namespace std;
using namespace Eigen;

const float DEG_2_RAD = M_PI / 180.0;

geometry_msgs::PoseWithCovarianceStamped Calibration_d1;
geometry_msgs::PoseWithCovarianceStamped Calibration_f1;
geometry_msgs::PoseWithCovarianceStamped Calibration_d2;
geometry_msgs::PoseWithCovarianceStamped Calibration_f2;
geometry_msgs::PoseWithCovarianceStamped Calibration_d3;
geometry_msgs::PoseWithCovarianceStamped Calibration_f3;

std_msgs::Header header_d;
std_msgs::Header header_f;

float odomX, odomY, odomZ;
//float d_x_err, d_y_err, d_z_err;
vector<vector<float>> downward_pos_error;
vector<vector<float>> downward_pos;
float f_x_err, f_y_err, f_z_err;
float calf_x, calf_y, calf_z;

vector<Eigen::Quaterniond> q_d;
Eigen::Quaterniond q_f;
 
float x_f, y_f, z_f, w_f, i_f, j_f, k_f;
vector<vector<float>> forward_tags(3, vector<float>(7));  // (x, y, z, w, i, j, k)
vector<vector<float>> downward_tags(3, vector<float>(7));

void OdeCallback(const geometry_msgs::Pose::ConstPtr &msg)
{

	odomX = msg->position.x;
	odomY = msg->position.y;
    odomZ = msg->position.z;

}

void IrisPositionDownword(vector<Eigen::Quaterniond> q_d)
{
	vector<float> temp_pos;

	for(unsigned int num = 0; num < q_d.size(); num++) {
		temp_pos = { float(q_d[num].x() + 0), float(q_d[num].y() + 0), float(q_d[num].z() + 0.01 + 0.05) };
		downward_pos.push_back(temp_pos);
	}

	for(unsigned int num = 0; num < q_d.size(); num++) {
		switch(num) {
			case 0:    // first downward pose sensor
				// Message of downward camera
				Calibration_d1.header.stamp = header_d.stamp;
				Calibration_d1.header.frame_id = "iris1/odometry_sensor1";

				Calibration_d1.pose.pose.position.x = downward_pos[num][0];
				Calibration_d1.pose.pose.position.y = downward_pos[num][1];
				Calibration_d1.pose.pose.position.z = downward_pos[num][2];

				downward_pos_error.push_back( { abs(odomX-downward_pos[num][0]), abs(odomY-downward_pos[num][1]), abs(odomZ-downward_pos[num][2]) } );

				break;
			case 1:   // second downward pose sensor
				// Message of downward camera
				Calibration_d2.header.stamp = header_d.stamp;
				Calibration_d2.header.frame_id = "iris1/odometry_sensor1";

				Calibration_d2.pose.pose.position.x = downward_pos[num][0];
				Calibration_d2.pose.pose.position.y = downward_pos[num][1];
				Calibration_d2.pose.pose.position.z = downward_pos[num][2];

				downward_pos_error.push_back( { abs(odomX-downward_pos[num][0]), abs(odomY-downward_pos[num][1]), abs(odomZ-downward_pos[num][2]) } );

				break;
			case 2:   // third downward pose sensor
				// Message of downward camera
				Calibration_d3.header.stamp = header_d.stamp;
				Calibration_d3.header.frame_id = "iris1/odometry_sensor1";

				Calibration_d3.pose.pose.position.x = downward_pos[num][0];
				Calibration_d3.pose.pose.position.y = downward_pos[num][1];
				Calibration_d3.pose.pose.position.z = downward_pos[num][2];

				downward_pos_error.push_back( { abs(odomX-downward_pos[num][0]), abs(odomY-downward_pos[num][1]), abs(odomZ-downward_pos[num][2]) } );

				break;
			default:
				break;
		}
	}
	
}

void Print_forward_error(float x, float y, float z)
{
	calf_x = 3 - (z + 0.01) - 0.1;
	calf_y = y;
	calf_z = 3 + x;

	// Message of forward camera
	Calibration_f1.header.stamp = header_f.stamp;
	Calibration_f1.header.frame_id = "iris1/odometry_sensor1";
	Calibration_f1.pose.pose.position.x = calf_x;
	Calibration_f1.pose.pose.position.y = calf_y;
	Calibration_f1.pose.pose.position.z = calf_z;

	//cout << "calbration_f_x: " << calf_x << ", calbration_f_y: " << calf_y << ", calbration_f_z: " << calf_z << endl;

	f_x_err = abs(odomX - calf_x);
	f_y_err = abs(odomY - calf_y);
	f_z_err = abs(odomZ - calf_z);

	//cout << "forward_error_x: " << f_x_err << ", forward_error_y: " << f_y_err 
	//	<< ", forward_error_z: " << f_z_err << endl << endl;
}

Eigen::Quaterniond Quaterion_calcutaion(float i, float j, float k, float w, float x, float y, float z)
{
	Eigen::Quaterniond q1;
	Eigen::Quaterniond q2;
	Eigen::Quaterniond q3;

	q1.x() = i; q1.y() = j; q1.z() = k; q1.w() = w;
	q2.x() = x; q2.y() = y; q2.z() = z; q2.w() = 0;

	q3 = q1.inverse() * q2 * q1;

	return q3;
}

void downCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{

	Eigen::Quaterniond temp_q;
	
	if(msg->detections.size() > 0) {
		header_d = msg->detections[0].pose.header;

		for(unsigned int count = 0; count < msg->detections.size(); count++) {
			downward_tags[count][0] = msg->detections[count].pose.pose.pose.position.x;
			downward_tags[count][1] = msg->detections[count].pose.pose.pose.position.y;
			downward_tags[count][2] = msg->detections[count].pose.pose.pose.position.z;
			downward_tags[count][3] = msg->detections[count].pose.pose.pose.orientation.w;
			downward_tags[count][4] = msg->detections[count].pose.pose.pose.orientation.x;
			downward_tags[count][5] = msg->detections[count].pose.pose.pose.orientation.y;
			downward_tags[count][6] = msg->detections[count].pose.pose.pose.orientation.z;

			temp_q = Quaterion_calcutaion(downward_tags[count][4], downward_tags[count][5], downward_tags[count][6], downward_tags[count][3], downward_tags[count][0], downward_tags[count][1], downward_tags[count][2]);
			q_d.push_back(temp_q);
		}
		
		IrisPositionDownword(q_d);
	}
}

void forCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{
	if(msg->detections.size() > 0) {
		header_f = msg->detections[0].pose.header;
		x_f = msg->detections[0].pose.pose.pose.position.x;
		y_f = msg->detections[0].pose.pose.pose.position.y;
		z_f = msg->detections[0].pose.pose.pose.position.z;
		i_f = msg->detections[0].pose.pose.pose.orientation.x;
		j_f = msg->detections[0].pose.pose.pose.orientation.y;
		k_f = msg->detections[0].pose.pose.pose.orientation.z;
		w_f = msg->detections[0].pose.pose.pose.orientation.w;

		q_f = Quaterion_calcutaion(i_f, j_f, k_f, w_f, x_f, y_f, z_f);

		Print_forward_error(-q_f.x(), -q_f.y(), -q_f.z());
	} 
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "apriltag_coordinate_transformation");
	ros::NodeHandle nh;

	ROS_INFO("Start tranfering");

	// Node setting
	ros::Publisher Pose_detect_forward1;
	ros::Publisher Pose_detect_downward1;
	ros::Publisher Pose_detect_forward2;
	ros::Publisher Pose_detect_downward2;
	ros::Publisher Pose_detect_forward3;
	ros::Publisher Pose_detect_downward3;
	ros::Subscriber ode_sub;
	ros::Subscriber downward_tag;
	ros::Subscriber forward_tag;

	ros::Rate r(10000); // 100us

	// Variable setting
	Pose_detect_downward1 = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/iris1/pose_d1", 100);
	Pose_detect_forward1 = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/iris1/pose_f1", 100);
	Pose_detect_downward2 = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/iris1/pose_d2", 100);
	Pose_detect_forward2 = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/iris1/pose_f2", 100);
	Pose_detect_downward3 = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/iris1/pose_d3", 100);
	Pose_detect_forward3 = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/iris1/pose_f3", 100);

	forward_tag = nh.subscribe("/forward/tag_detections", 100, forCallback);
	downward_tag = nh.subscribe("/downward/tag_detections", 100, downCallback);
	ode_sub = nh.subscribe("/iris1/odometry_sensor1/pose", 100, OdeCallback);

	while (ros::ok())
	{
		Pose_detect_downward1.publish(Calibration_d1);
		Pose_detect_forward1.publish(Calibration_f1);
		Pose_detect_downward2.publish(Calibration_d2);
		Pose_detect_forward2.publish(Calibration_f2);
		Pose_detect_downward3.publish(Calibration_d3);
		Pose_detect_forward3.publish(Calibration_f3);
	
		ros::spinOnce();
		r.sleep();
	}
	
	return 0;
}
