#include"ros/ros.h"
#include"omnivelma_msgs/Vels.h"
#include"omnivelma_msgs/SetVelocity.h"
#include<iostream>
#include<string>
#include<functional>

const double risingVelocity = 30;

void velocityCallback(const omnivelma_msgs::Vels::ConstPtr& message, omnivelma_msgs::Vels& velocities, ros::Publisher& publisher){
	ROS_INFO("STPT rr=%f, rl=%f, fr=%f, fl=%f", message->rr, message->rl, message->fr, message->fl);
	const double lastVelRR = velocities.rr;
	const double lastVelRL = velocities.rl;
	const double lastVelFR = velocities.fr;
	const double lastVelFL = velocities.fl;

	const double diffRR = (message->rr - lastVelRR)/risingVelocity;
	const double diffRL = (message->rl - lastVelRL)/risingVelocity;
	const double diffFR = (message->fr - lastVelFR)/risingVelocity;
	const double diffFL = (message->fl - lastVelFL)/risingVelocity;

	ros::Rate loopRate(10);
	for(int i = 0; i < risingVelocity; ++i){
		velocities.rr += diffRR;
		velocities.rl += diffRL;
		velocities.fr += diffFR;
		velocities.fl += diffFL;
		ROS_INFO("Vels rr=%f, rl=%f, fr=%f, fl=%f", velocities.rr, velocities.rl, velocities.fr, velocities.fl);
		publisher.publish(velocities);
		loopRate.sleep();
	}
}

int main(int argc, char **argv){
	ros::init(argc, argv, "omnivelma_driver");
	ros::NodeHandle node;

	omnivelma_msgs::Vels state;
	ros::Publisher velocityPublisher = node.advertise<omnivelma_msgs::Vels>("/velma/omnivelma/vels", 1000);

	/* accept velocity */
	ros::Subscriber velocitySubscriber = node.subscribe<omnivelma_msgs::Vels>("/omnivelma/velocity", 1000, boost::bind(velocityCallback, _1, std::ref(state), std::ref(velocityPublisher)));

	/* velocity trapezoid publisher*/

	ros::Rate loopRate(10);
	unsigned int counter = 0;

	while (ros::ok()) {
		velocityPublisher.publish(state);
		ros::spinOnce();
		loopRate.sleep();
		++counter;
	}

	return 0;
}

