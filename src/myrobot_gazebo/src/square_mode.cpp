#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
//#include "turtlesim/Pose.h"
#include <sstream>

using namespace std;

ros::Publisher velocity_publisher;
//ros::Subscriber pose_subscriber;

const double PI = 3.14159265359;

// method to move the robot straight
void move(double speed, double distance, bool isForward);
void rotate(double angular_speed, double angle, bool clockwise);
double degrees2radians(double angle_in_degrees);

int main(int argc, char **argv)
{

	// initiate new ROS node called myrobot_cleaner
	ros::init(argc, argv, "myrobot_cleaner");
	ros::NodeHandle n;
	double speed, angular_speed;
	double distance, angle;
	bool isForward, clockwise;

	velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	
	cout << "Enter speed: ";
	cin >> speed;
	cout << "Enter distance: ";
	cin >> distance;
	cout << "Enter Forward?: ";
	cin >> isForward;
	move (speed, distance, isForward);

	cout << "Enter angular velocity (degree/sec): ";
	cin >> angular_speed;
	cout << "Enter desired angle (degrees): ";
	cin >> angle;
	cout << "Clockwise?: ";
	cin >> clockwise;
	rotate(degrees2radians(angular_speed), degrees2radians(angle), clockwise);

	ros::spin();
	return 0;

}

void move(double speed, double distance, bool isForward)
{
	geometry_msgs::Twist vel_msg;
	// distance = speed * time
	
	// set a random linear velocity in the x-axis
	if(isForward)
		vel_msg.linear.x = abs(speed);
	else
		vel_msg.linear.x = -abs(speed);
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;

	// set a random angular velocity
	vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = 0;

	// t0: current time
	double t0 = ros::Time::now().toSec();
	double current_distance = 0.0;
	ros::Rate loop_rate(10);
	do {
		velocity_publisher.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_distance = speed * (t1 - t0);
		ros::spinOnce();
		loop_rate.sleep();
	} while (current_distance <= distance);
	vel_msg.linear.x = 0;
	velocity_publisher.publish(vel_msg);
	// loop
	// publish the velocity
	// estimate the current_distance = speed * (t1-t0)
	// current_distance_moved_by_robot <= distance
}

void rotate(double angular_speed, double relative_angle, bool clockwise)
{
	geometry_msgs::Twist vel_msg;
	//set a random linear velocity in the x-axis
	vel_msg.linear.x = 0;
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;
	//set a random angular velocity in the z-axis
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;

	if(clockwise)
		vel_msg.angular.z = -abs(angular_speed);
	else
		vel_msg.angular.z = abs(angular_speed);
	double current_angle = 0.0;
	double t0 = ros::Time::now().toSec();
	ros::Rate loop_rate(10);
	do{
		velocity_publisher.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_angle = angular_speed * (t1 - t0);
		ros::spinOnce();
		loop_rate.sleep();
	}while(current_angle < relative_angle);
	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg);
}

double degrees2radians(double angle_in_degrees)
{
	return angle_in_degrees * PI / 180.0;
}

