# subscriber_speed_test

This ROS package benchmarks how long it takes for ROS subscribers to connect to publishers and start exchanging messages.

## Building
```bash
mkdir -p ws/src && cd ws/src
git clone https://github.com/stevegolton/ros_subscriber_speed_test.git
cd ..
source /opt/ros/noetic/setup.bash
catkin_make
```

## Running
```bash
source devel/setup.bash
roslaunch subscriber_speed_test run_test.launch
```

The test will start a server node and a client node. The server starts various services (publishers, service servers and action servers) and spins forever. The client comes up, waits for an arbitrary amount of time for the server to come up, and then performs a bunch of tests to work out how long it takes to connect to various services/publishers on the server node.

These are the results I get on a relatively modern Intel Core i5 desktop machine using Ubuntu 20.04 and ROS Noetic. Nodes are running on the same machine so are connecting over the loopback interface.

```
Allowing ample time for the server to come up...
Started server
Starting tests...
Average time spent in service_client.waitForExistence = 0.000384 seconds
Average time spent in topic::waitForMessage = 0.198353 seconds
Average time spent in action_client.waitForServer = 0.231182 seconds
Testing complete
```
