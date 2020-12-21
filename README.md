# subscriber_speed_test

This ROS package benchmarks how long it takes for ROS subscribers to connect to publishers and start exchanging messages.

## Building

Make sure you have ROS installed and sourced. Tested using ROS Noetic.

```bash
mkdir -p ws/src && cd ws/src
git clone https://github.com/stevegolton/ros_subscriber_speed_test.git
cd ..
catkin_make
```

## Running

```bash
source devel/setup.bash
roslaunch subscriber_speed_test run_test_roscpp.launch
roslaunch subscriber_speed_test run_test_rospy.launch
```

The test will start a server node and a client node. The server starts various services (publishers, service servers and action servers) and spins forever. The client comes up, waits for an arbitrary amount of time for the server to come up, and then performs a bunch of tests to work out how long it takes to connect to various services/publishers on the server node.

These are the results I get on a relatively modern Intel Core i5 desktop machine using Ubuntu 20.04 and ROS Noetic. Nodes are running on the same machine so are connecting over the loopback interface.

run_test_roscpp.launch
```
Allowing ample time for the server to come up...
Starting tests...
Average time spent in service_client.waitForExistence = 0.000481 seconds
Average time spent in topic::waitForMessage = 0.198579 seconds
Average time spent in topic::waitForMessage (presub) = 0.000958 seconds
Average time spent in action_client.waitForServer = 0.240173 seconds
Average time spent in action_client.waitForServer (presub) = 0.102728 seconds
Average time spent in action_client.waitForServer (prewait) = 0.000002 seconds
Average time spent waiting for TF = 0.187526 seconds
Average time spent waiting for TF (presub) = 0.007806 seconds
Testing complete
```

run_test_rospy.launch
```
Allowing ample time for the server to come up...
Starting tests...
Average time spent in rospy.wait_for_message = 0.012007 seconds
Average time spent in action_client.wait_for_server = 0.002346 seconds
Average time spent waiting for TF = 0.010064 seconds
Testing complete
```
