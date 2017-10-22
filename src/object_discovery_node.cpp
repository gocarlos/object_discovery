/*
 * object_discovery_node.cpp
 *
 *  Created on: 2016.11.07
 *      Author: Carlos Gomes
 *	 Institute: ETH Zurich, RSL
 *
 * This is the main ROS node of the object_discovery package.
 * It provides services which when called trigger the
 * segmentation of a given point cloud. It also subscribes
 * to point cloud topics, so point clouds which should be
 * segmented can also be gathered from a ROS topic.
 */

#include <glog/logging.h>
#include <ros/ros.h>
#include "object_discovery/object_discovery_ros_interface.hpp"

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = 1;
  FLAGS_colorlogtostderr = 1;
  FLAGS_logbufsecs = 1;
  ros::init(argc, argv, "object_discovery");
  ros::NodeHandle nodeHandle("~");
  object_discovery::ObjectDiscoveryRosInterface objectDiscoveryRosInterface(
      nodeHandle);
  ros::spin();
  return 0;
}
