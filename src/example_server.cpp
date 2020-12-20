#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <actionlib/server/simple_action_server.h>
#include <subscriber_speed_test/EmptyAction.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

void timer_handler(const ros::TimerEvent &)
{
	static tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped transformStamped;

	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = "world";
	transformStamped.child_frame_id = "example";
	transformStamped.transform.rotation.w = 1.0;

	br.sendTransform(transformStamped);
}

bool service_handler(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	return true;
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "example_server");

	ros::NodeHandle nh;
	auto timer = nh.createTimer(ros::Duration(0.01), timer_handler);
	auto server = nh.advertiseService("service", service_handler);
	auto latched_topic = nh.advertise<std_msgs::Empty>("latched_topic", 1, true);

	std_msgs::Empty msg;
	latched_topic.publish(msg);

	actionlib::SimpleActionServer<subscriber_speed_test::EmptyAction> action_server(nh, "action", false);
	action_server.start();

	printf("Started server");
	ros::spin();

	return 0;
}
