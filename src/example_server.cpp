#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <actionlib/server/simple_action_server.h>
#include <subscriber_speed_test/EmptyAction.h>

bool service_handler(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    return true;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "example_server");

    ros::NodeHandle nh;
    auto server = nh.advertiseService("service", service_handler);
    auto latched_topic = nh.advertise<std_msgs::Empty>("latched_topic", 1, true);

    std_msgs::Empty msg;
    latched_topic.publish(msg);

    actionlib::SimpleActionServer<subscriber_speed_test::EmptyAction> action_server(nh, "action", false);
    action_server.start();

    ROS_INFO("Started server");
    ros::spin();

    return 0;
}
