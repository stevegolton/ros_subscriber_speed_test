#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <actionlib/client/simple_action_client.h>
#include <subscriber_speed_test/EmptyAction.h>
#include <tf2_ros/transform_listener.h>

const int COUNT = 32;

void test_service_waitForExistence()
{
	ros::NodeHandle nh;
	ros::Duration total_time;
	for (int i = 0; i < COUNT; i++)
	{
		auto before = ros::Time::now();
		auto client = nh.serviceClient<std_srvs::Empty>("service");
		client.waitForExistence();
		total_time += (ros::Time::now() - before);
	}

	printf("Average time spent in service_client.waitForExistence = %f seconds\n", total_time.toSec() / COUNT);
}

void test_topic_waitForMessage()
{
	ros::NodeHandle nh;
	ros::Duration total_time;
	for (int i = 0; i < COUNT; i++)
	{
		auto before = ros::Time::now();
		ros::topic::waitForMessage<std_msgs::Empty>("latched_topic", nh);
		total_time += (ros::Time::now() - before);
	}

	printf("Average time spent in topic::waitForMessage = %f seconds\n", total_time.toSec() / COUNT);
}

void test_topic_presub_waitForMessage()
{
	ros::NodeHandle nh;
	ros::Subscriber presub = nh.subscribe<std_msgs::Empty>("latched_topic", 1, [](std_msgs::EmptyConstPtr) {});
	ros::Duration(1.0).sleep();

	ros::Duration total_time;
	for (int i = 0; i < COUNT; i++)
	{
		auto before = ros::Time::now();
		ros::topic::waitForMessage<std_msgs::Empty>("latched_topic", nh);
		total_time += (ros::Time::now() - before);
	}

	printf("Average time spent in topic::waitForMessage (presub) = %f seconds\n", total_time.toSec() / COUNT);
}

void test_action_waitForService()
{
	ros::NodeHandle nh;
	ros::Duration total_time;
	for (int i = 0; i < COUNT; i++)
	{
		auto before = ros::Time::now();
		actionlib::SimpleActionClient<subscriber_speed_test::EmptyAction> action_client("action", true);
		action_client.waitForServer();
		total_time += (ros::Time::now() - before);
	}

	printf("Average time spent in action_client.waitForServer = %f seconds\n", total_time.toSec() / COUNT);
}

void test_action_presub_waitForService()
{
	ros::NodeHandle nh;
	actionlib::SimpleActionClient<subscriber_speed_test::EmptyAction> presub_action_client("action", true);
	presub_action_client.waitForServer();

	ros::Duration total_time;
	for (int i = 0; i < COUNT; i++)
	{
		auto before = ros::Time::now();
		actionlib::SimpleActionClient<subscriber_speed_test::EmptyAction> action_client("action", true);
		action_client.waitForServer();
		total_time += (ros::Time::now() - before);
	}

	printf("Average time spent in action_client.waitForServer (presub) = %f seconds\n", total_time.toSec() / COUNT);
}

void test_action_prewait_waitForService()
{
	ros::NodeHandle nh;
	ros::Duration total_time;
	for (int i = 0; i < COUNT; i++)
	{

		actionlib::SimpleActionClient<subscriber_speed_test::EmptyAction> action_client("action", true);
		action_client.waitForServer();

		auto before = ros::Time::now();
		action_client.waitForServer();
		total_time += (ros::Time::now() - before);
	}

	printf("Average time spent in action_client.waitForServer (prewait) = %f seconds\n", total_time.toSec() / COUNT);
}

void test_tf2_sub()
{
	ros::Duration total_time;
	for (int i = 0; i < COUNT; i++)
	{
		tf2_ros::Buffer tfBuffer;
		tf2_ros::TransformListener listener(tfBuffer);

		geometry_msgs::TransformStamped transform;
		auto before = ros::Time::now();
		while (true)
		{
			try
			{
				transform = tfBuffer.lookupTransform("world", "example", ros::Time(0));
				break;
			}
			catch (tf2::TransformException ex)
			{
				ros::Duration(0.001).sleep();
			}
		}
		total_time += (ros::Time::now() - before);
	}
	printf("Average time spent waiting for TF = %f seconds\n", total_time.toSec() / COUNT);
}

void test_tf2_presub_sub()
{
	tf2_ros::Buffer tfBuffer_presub;
	tf2_ros::TransformListener listener_presub(tfBuffer_presub);
	ros::Duration(1.0).sleep();

	ros::Duration total_time;
	for (int i = 0; i < COUNT; i++)
	{
		tf2_ros::Buffer tfBuffer;
		tf2_ros::TransformListener listener(tfBuffer);

		geometry_msgs::TransformStamped transform;
		auto before = ros::Time::now();
		while (true)
		{
			try
			{
				transform = tfBuffer.lookupTransform("world", "example", ros::Time(0));
				break;
			}
			catch (tf2::TransformException ex)
			{
				ros::Duration(0.001).sleep();
			}
		}
		total_time += (ros::Time::now() - before);
	}
	printf("Average time spent waiting for TF (presub) = %f seconds\n", total_time.toSec() / COUNT);
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "example_client");

	ros::NodeHandle nh;
	printf("Allowing ample time for the server to come up...\n");
	ros::Duration(3).sleep();

	printf("Starting tests...\n");
	test_service_waitForExistence();
	test_topic_waitForMessage();
	test_topic_presub_waitForMessage();
	test_action_waitForService();
	test_action_presub_waitForService();
	test_action_prewait_waitForService();
	test_tf2_sub();
	test_tf2_presub_sub();
	printf("Testing complete\n");

	return 0;
}
