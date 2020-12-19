#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <actionlib/client/simple_action_client.h>
#include <subscriber_speed_test/EmptyAction.h>

void test_service_waitForExistence()
{
    ros::NodeHandle nh;
    ros::Duration total_time;
    const int COUNT = 32;
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
    const int COUNT = 32;
    for (int i = 0; i < COUNT; i++)
    {
        auto before = ros::Time::now();
        ros::topic::waitForMessage<std_msgs::Empty>("latched_topic", nh);
        total_time += (ros::Time::now() - before);
    }

    printf("Average time spent in topic::waitForMessage = %f seconds\n", total_time.toSec() / COUNT);
}

void test_action_waitForService()
{
    ros::NodeHandle nh;
    ros::Duration total_time;
    const int COUNT = 32;
    for (int i = 0; i < COUNT; i++)
    {
        auto before = ros::Time::now();
        actionlib::SimpleActionClient<subscriber_speed_test::EmptyAction> action_client("action", true);
        action_client.waitForServer();
        total_time += (ros::Time::now() - before);
    }

    printf("Average time spent in action_client.waitForServer = %f seconds\n", total_time.toSec() / COUNT);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "example_client");

    ros::NodeHandle nh;
    printf("Allowing ample time for the server to come up\n");
    ros::Duration(3).sleep();

    printf("Starting tests...\n");
    test_service_waitForExistence();
    test_topic_waitForMessage();
    test_action_waitForService();
    printf("Testing complete\n");

    return 0;
}
