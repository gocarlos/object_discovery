/*
 * object.cpp
 *
 *  Created on: Dec 11, 2016
 *      Author: gocarlos
 */

#include "object_discovery/object.hpp"

namespace object_discovery {

Object::Object() : is_supporting_surface(false), object_id(0) {
  position.setZero();
  orientation.setIdentity();
  scale.setZero();
}

pcl::PointCloud<pcl::PointNormal>& Object::getCloud() { return object_cloud; }

void Object::setCloud(
    const pcl::PointCloud<pcl::PointNormal>::ConstPtr& cloud) {
  CHECK_NOTNULL(cloud.get());
  CHECK_GT(cloud->points.size(), 0);
  object_cloud = *cloud;
}

shape_msgs::Mesh& Object::getMesh() { return object_mesh; }
void Object::setMesh(const shape_msgs::Mesh::ConstPtr& mesh) {
  CHECK_NOTNULL(mesh.get());
  CHECK_GT(mesh->triangles.size(), 0);
  CHECK_GT(mesh->vertices.size(), 3);

  object_mesh = *mesh;
}

Eigen::Vector3f& Object::getPosition() { return position; }
void Object::setPosition(const Eigen::Vector3f& pos) { position = pos; }

Eigen::Quaternionf& Object::getOrientation() { return orientation; }
void Object::setOrientation(const Eigen::Quaternionf& quaternion) {
  orientation = quaternion;
}

Eigen::Vector3f& Object::getScale() { return scale; }
void Object::setScale(const Eigen::Vector3f& s) { scale = s; }

int Object::getObjectId() { return object_id; }
void Object::setObjectId(const size_t& id) { object_id = id; }

bool Object::isSupportingSurface() { return is_supporting_surface; }
void Object::setSupportingSurface(const bool& isSupporting) {
  is_supporting_surface = isSupporting;
}

std::vector<float>& Object::getObjectness() { return objectness_; }
void Object::setObjectness(const std::vector<float>& objectness) {
  objectness_ = objectness;
}

}  // namespace object_discovery
