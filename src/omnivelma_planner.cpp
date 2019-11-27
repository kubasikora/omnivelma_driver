#include "ros/ros.h"
#include <iostream>
#include <vector>
#include <costmap_2d/costmap_2d_ros.h>
#include <global_planner/planner_core.h>
//#include <dwa_local_planner/dwa_planner_ros.h>
#include <teb_local_planner/teb_local_planner_ros.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/buffer.h"
#include "omnivelma_driver/STPT.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include <tf2/LinearMath/Quaternion.h>
#include <rotate_recovery/rotate_recovery.h>
#include <cmath>


// global planner
std::unique_ptr<tf2_ros::Buffer> globalBuffer;
std::unique_ptr<tf2_ros::TransformListener> tfGlobal;
std::unique_ptr<costmap_2d::Costmap2DROS> globalCostmap;
std::unique_ptr<global_planner::GlobalPlanner> globalPlanner;

// local planner
std::unique_ptr<tf2_ros::Buffer> localBuffer;
std::unique_ptr<tf2_ros::TransformListener> tfLocal;
std::unique_ptr<costmap_2d::Costmap2DROS> localCostmap;
//std::unique_ptr<dwa_local_planner::DWAPlannerROS> localPlanner;
std::unique_ptr<teb_local_planner::TebLocalPlannerROS> localPlanner;

// nodes
ros::ServiceServer stptService;
ros::Publisher velocityPublisher;



/* -----------------  FUNCTION DECLARATIONS  ------------------------ */

/*
* callback do serwera serwisu pozycji zadanej robota
*/
bool stptServiceCallback(omnivelma_driver::STPT::Request &req, omnivelma_driver::STPT::Response &res);

/*
* callback subscribera danych odometrii
*/
void odomCallback(const nav_msgs::Odometry::ConstPtr&  msg);

/*
* planowanie ruchu do setpoint
*/
std::string planMove(geometry_msgs::PoseStamped &stpt);

/*
* recovery behaviour
*/
void rotateRecoveryBehavior();

/*
* get yaw from quaternion
*/
double thetaFromQuat(geometry_msgs::Quaternion &orientation);


int main(int argc, char* argv[]){
	ros::init(argc, argv, "omniplanner");
	ros::NodeHandle nodeHandle;


	globalBuffer.reset(new tf2_ros::Buffer(ros::Duration(10), true));
	tfGlobal.reset(new tf2_ros::TransformListener(*globalBuffer));
	globalCostmap.reset(new costmap_2d::Costmap2DROS("global_costmap", *globalBuffer));
	globalCostmap->start();
	globalPlanner.reset(new global_planner::GlobalPlanner("global_planner", globalCostmap->getCostmap(), "map"));


	localBuffer.reset(new tf2_ros::Buffer(ros::Duration(10), true));
	tfLocal.reset(new tf2_ros::TransformListener(*localBuffer));
	localCostmap.reset(new costmap_2d::Costmap2DROS("local_costmap", *localBuffer));
	//localPlanner.reset(new dwa_local_planner::DWAPlannerROS());
	localPlanner.reset(new teb_local_planner::TebLocalPlannerROS());
	localPlanner->initialize("local_planner", localBuffer.get(), localCostmap.get());

	stptService = nodeHandle.advertiseService("omniplanner/go_to_stpt", stptServiceCallback);
	velocityPublisher = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

	ROS_INFO_STREAM("Planner module initialized properly...");
	ros::spin();

	return 0;
}


bool stptServiceCallback(omnivelma_driver::STPT::Request& request, omnivelma_driver::STPT::Response& response){
	tf2::Quaternion q;
	q.setRPY(0.0, 0.0, request.pose.theta);

	std::cout << "New setpoint: " << std::endl;
	std::cout << "x = " << request.pose.x << ", y = " << request.pose.y << ", yaw: " << request.pose.theta << std::endl;

	geometry_msgs::PoseStamped stpt;
	stpt.header.frame_id = "map";
	stpt.header.stamp = ros::Time::now();
	stpt.pose.position.x = request.pose.x;
	stpt.pose.position.y = request.pose.y;
	stpt.pose.position.z = 0.0;
	stpt.pose.orientation.x = q.getX();
	stpt.pose.orientation.y = q.getY();
	stpt.pose.orientation.z = q.getZ();
	stpt.pose.orientation.w = q.getW();

	try { 
		response.result = planMove(stpt);
	} catch(...) {
		response.result = "Exception occured";
		return false;
	}

	return true;
}

std::string planMove(geometry_msgs::PoseStamped& setpoint){
	bool correctFinish = false;
	bool noPlanError = false;
	int noPlanCount = 0;
	ros::Rate rosRate(10);

	std::vector<geometry_msgs::PoseStamped> localPath;
	geometry_msgs::Twist twist;
	geometry_msgs::PoseStamped pose;
	geometry_msgs::PoseStamped start;

	pose.header.frame_id = "map";
	pose.header.stamp = ros::Time::now();
	start.header.frame_id = "map";
	start.header.stamp = ros::Time::now();
	
	if(!globalCostmap->getRobotPose(start))
		return "Could not acquire robot position from global costmap";

	if(!globalPlanner->makePlan(start, setpoint, localPath))
		return "Could not make global plan";

	if(!localPlanner->setPlan(localPath))
		return "Could not pass global plan to local planner";

	globalPlanner->publishPlan(localPath);

	while(ros::ok()){
		if(localPlanner->isGoalReached()){
			std::cout << "goal reached" << std::endl;
			correctFinish = true;
			break;
		}

		if(!localPlanner->computeVelocityCommands(twist)){
			// recovery situation - could not compute new vel cmd
			std::cout << "Could not compute next velocity message, executing recovery behaviour..." << std::endl;
			++noPlanCount;
			switch(noPlanCount){
				case 1:
					std::cout << "Try 1: Clearing local costmap and run simple rotate behaviour" << std::endl;
					localCostmap->resetLayers();
					//rotateRecoveryBehavior();
					ros::spinOnce();
					continue;
				case 2:
					std::cout << "Try 2: Resetting global plan" << std::endl;
					if(!globalCostmap->getRobotPose(start))
						return "Could not acquire robot position from global costmap";

					if(!globalPlanner->makePlan(start, setpoint, localPath))
						return "Could not make global plan";

					if(!localPlanner->setPlan(localPath))
						return "Could not pass global plan to local planner";

					globalPlanner->publishPlan(localPath);
					break;
				default:
					std::cout << "Failed to recover, abort mission" << std::endl;
					noPlanError = true;
			};

			if(noPlanError)
				break;
		} else {
			noPlanCount = 0;
		}

		velocityPublisher.publish(twist);
		ros::spinOnce();
		rosRate.sleep();
	}

	geometry_msgs::Twist stopCommand;
	velocityPublisher.publish(stopCommand);

	if(noPlanError){
		return "No local plan could be found";
	}

	if(correctFinish){
		return "Setpoint reached";
	}

	return "OK";
}


void rotateRecoveryBehavior(){
	geometry_msgs::Twist twist;
	ros::Rate rosRate(10);
	double startingTheta;
	geometry_msgs::PoseStamped start;
	start.header.frame_id = "map";
	start.header.stamp = ros::Time::now();

	if(!globalCostmap->getRobotPose(start)){
		std::cout << "Rotate recovery behaviour failed: could not get robot position" << std::endl;
		return;
	}

	startingTheta = thetaFromQuat(start.pose.orientation);
	const double recoveryAngularVelocity = 0.2;
	twist.angular.z = recoveryAngularVelocity;
	const unsigned int maxTries = 15;
	for(unsigned n = 0; n < maxTries; ++n){
		velocityPublisher.publish(twist);
		ros::spinOnce();
		rosRate.sleep();
	}
	do {
		if(!globalCostmap->getRobotPose(start)){
			std::cout << "Rotate recovery behaviour failed: could not get robot position" << std::endl;
			return;
		}
		velocityPublisher.publish(twist);
		ros::spinOnce();
		rosRate.sleep();
	}
	while( 0.01 < fabs( startingTheta-thetaFromQuat(start.pose.orientation) ) );
    twist.angular.z = 0.0;
    velocityPublisher.publish(twist);
}

double thetaFromQuat(geometry_msgs::Quaternion &orientation)
{
    double roll, pitch, yaw;
    tf2::Quaternion quat( orientation.x, orientation.y, orientation.z, orientation.w );
    tf2::Matrix3x3 matrix(quat);
    matrix.getRPY(roll, pitch, yaw);
    return yaw;
}