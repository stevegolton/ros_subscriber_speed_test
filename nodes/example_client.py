#!/usr/bin/env python

import rospy
import actionlib
from std_msgs.msg import Empty
from subscriber_speed_test.msg import EmptyAction
import tf2_ros


def test_topic_waitForMessage():
    total_time = rospy.Duration()
    for _ in range(COUNT):
        before = rospy.Time.now()
        rospy.wait_for_message("latched_topic", Empty)
        total_time += rospy.Time.now() - before

    print("Average time spent in rospy.wait_for_message = %f seconds" % (total_time.to_sec() / COUNT))


def test_action_waitForService():
    total_time = rospy.Duration()
    for _ in range(COUNT):
        before = rospy.Time.now()
        client = actionlib.SimpleActionClient('action', EmptyAction)
        client.wait_for_server()
        total_time += rospy.Time.now() - before

    print("Average time spent in action_client.wait_for_server = %f seconds" % (total_time.to_sec() / COUNT));


def test_tf2_sub():
    total_time = rospy.Duration()
    for _ in range(COUNT):
        before = rospy.Time.now()
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        while True:
            try:
                trans = tfBuffer.lookup_transform('world', 'example', rospy.Time())
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.sleep(0.001)

        total_time += rospy.Time.now() - before

    print("Average time spent waiting for TF = %f seconds" % (total_time.to_sec() / COUNT))


rospy.init_node('example_client', anonymous=True)
COUNT = 32

print("Allowing ample time for the server to come up...")
rospy.sleep(3.)

print("Starting tests...");
test_topic_waitForMessage()
test_action_waitForService()
test_tf2_sub()
print("Testing complete");
