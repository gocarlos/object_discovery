/*
 * object.hpp
 *
 *  Created on: Dec 11, 2016
 *      Author: gocarlos
 */

#pragma once

#include <glog/logging.h>

// PCL
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/surface/mls.h>
#include <pcl/impl/point_types.hpp>  // PointNormal type

// ROS
#include <shape_msgs/Mesh.h>

#include <Eigen/Dense>

#include "object_discovery/utils.h"

namespace object_discovery {

class Object {
 public:
  Object();

  pcl::PointCloud<pcl::PointNormal>& getCloud();
  void setCloud(const pcl::PointCloud<pcl::PointNormal>::ConstPtr& cloud);

  shape_msgs::Mesh& getMesh();
  void setMesh(const shape_msgs::Mesh::ConstPtr& mesh);

  Eigen::Vector3f& getPosition();
  void setPosition(const Eigen::Vector3f& pos);

  Eigen::Quaternionf& getOrientation();
  void setOrientation(const Eigen::Quaternionf& quaternion);

  Eigen::Vector3f& getScale();
  void setScale(const Eigen::Vector3f& s);

  int getObjectId();
  void setObjectId(const size_t& id);

  bool isSupportingSurface();
  void setSupportingSurface(const bool& isSupporting);

  std::vector<float>& getObjectness();
  void setObjectness(const std::vector<float>& objectness);

 private:
  pcl::PointCloud<pcl::PointNormal> object_cloud;

  shape_msgs::Mesh object_mesh;

  Eigen::Vector3f position;

  Eigen::Quaternionf orientation;

  Eigen::Vector3f scale;

  std::vector<float> objectness_;

  bool is_supporting_surface;

  int object_id;
};

} /* namespace object_discovery */
