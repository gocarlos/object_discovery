// Author: Stephen Miller

#include "object_discovery/vis_wrapper.h"

#include <cstdlib>
#include <sstream>

#include <pcl/common/centroid.h>
#include <pcl/ros/conversions.h>
#include <pcl/visualization/impl/pcl_visualizer.hpp>

using namespace pcl;
using namespace pcl::visualization;

VisWrapper::VisWrapper(std::string name, bool pick_points, bool mouse_cb)
    : Lockable(),
      key_(0),
      name_(name),
      holding_on_(false),
      ppc_updated_(false),
      mouse_cb_(mouse_cb) {
  vis_.registerKeyboardCallback(&VisWrapper::keyboardCallback, *this);
  // if(pick_points){
  vis_.registerPointPickingCallback(&VisWrapper::pointPickingCallback, *this);
  //}
  if (mouse_cb_) {
    vis_.registerMouseCallback(&VisWrapper::mouseCallback, *this);
  }
  // Initialize random seed for naming
  srand(time(NULL));
}

// BUG: v.close still does not work.
// https://github.com/PointCloudLibrary/pcl/issues/172
void VisWrapper::close() {
  clearAll();
  vis_.close();
}

void VisWrapper::waitLoop(char exitKey) {
  while (waitKey() != exitKey) continue;
}

char VisWrapper::waitKey(int msec) {
  lock();
  runEveryWaitKey();
  key_ = 0;
  if (msec == 0)
    while (key_ == 0) vis_.spinOnce(1);
  else
    vis_.spinOnce(msec);
  unlock();
  return key_;
}

PointBW_t VisWrapper::waitPoint(std::string& handle, int& idx) {
  handle = "NONE";
  idx = -1;
  while (waitKey(10) != 'q') {
    if (ppc_updated_) {
      handle = ppc_handle_;
      idx = ppc_idx_;
      ppc_updated_ = false;
      return ppc_point_;
    }
  }
}
PointBW_t VisWrapper::waitPoint() {
  std::string handle;
  int idx;
  return waitPoint(handle, idx);
}

void VisWrapper::holdOn() { holding_on_ = true; }

void VisWrapper::holdOff() { holding_on_ = false; }

void VisWrapper::remove(std::string handle) {
  DisplayType type = type_by_handle_[handle];
  switch (type) {
    case BWCLOUD_TYPE:
      vis_.removePointCloud(handle);
      bwcloud_names_.erase(
          std::remove(bwcloud_names_.begin(), bwcloud_names_.end(), handle),
          bwcloud_names_.end());
      bwcloud_by_handle_[handle] = CloudBW_t::ConstPtr();
      break;
    case RGBCLOUD_TYPE:
      vis_.removePointCloud(handle);
      rgbcloud_names_.erase(
          std::remove(rgbcloud_names_.begin(), rgbcloud_names_.end(), handle),
          rgbcloud_names_.end());
      rgbcloud_by_handle_[handle] = CloudColor_t::ConstPtr();
      break;
    case KEYCLOUD_TYPE:
      vis_.removePointCloud(handle);
      keycloud_names_.erase(
          std::remove(keycloud_names_.begin(), keycloud_names_.end(), handle),
          keycloud_names_.end());
      keycloud_by_handle_[handle] = KeyCloud_t::ConstPtr();
      break;
    case MESH_TYPE:
      vis_.removePolygonMesh(handle);
      mesh_names_.erase(
          std::remove(mesh_names_.begin(), mesh_names_.end(), handle),
          mesh_names_.end());
      mesh_by_handle_[handle] = Mesh_t::ConstPtr();
      break;
  }
  handles_.erase(std::remove(handles_.begin(), handles_.end(), handle),
                 handles_.end());
}

void VisWrapper::clearAll() {
  for (size_t i = 0; i < bwcloud_names_.size(); ++i) {
    vis_.removePointCloud(bwcloud_names_[i]);
  }
  for (size_t i = 0; i < rgbcloud_names_.size(); ++i) {
    vis_.removePointCloud(rgbcloud_names_[i]);
  }
  for (size_t i = 0; i < keycloud_names_.size(); ++i) {
    vis_.removePointCloud(keycloud_names_[i]);
  }
  for (size_t i = 0; i < mesh_names_.size(); ++i) {
    vis_.removePolygonMesh(mesh_names_[i]);
  }
  bwcloud_names_.clear();
  rgbcloud_names_.clear();
  keycloud_names_.clear();
  mesh_names_.clear();
  type_by_handle_.clear();
  bwcloud_by_handle_.clear();
  keycloud_by_handle_.clear();
  mesh_by_handle_.clear();
  handles_.clear();
  color_by_handle_.clear();
}

void VisWrapper::keyboardCallback(
    const pcl::visualization::KeyboardEvent& event, void* cookie) {
  if (event.keyUp()) {
    key_ = event.getKeyCode();
    keyboardCallbackImpl(key_);
  }
}

void VisWrapper::mouseCallback(const pcl::visualization::MouseEvent& event,
                               void* cookie) {
  mouseCallbackImpl(event.getX(), event.getY(), event);
}

std::string VisWrapper::showCloud(CloudBW_t::ConstPtr pcd, int r, int g, int b,
                                  float radius, bool update) {
  if (update) lock();
  if (!holding_on_) clearAll();
  pcl::visualization::PointCloudColorHandlerCustom<PointBW_t> handler(pcd, r, g,
                                                                      b);
  std::string name = generate_bwcloud_name();
  vis_.addPointCloud(pcd, handler, name);
  vis_.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, radius, name);
  bwcloud_by_handle_[name] = pcd;
  if (update) {
    vis_.spinOnce();
    unlock();
  }
  color_by_handle_[name] = std::vector<int>(3);
  color_by_handle_[name][0] = r;
  color_by_handle_[name][1] = g;
  color_by_handle_[name][2] = b;
  return name;
}

std::string VisWrapper::showCloud(CloudColor_t::ConstPtr pcd, float radius,
                                  bool update) {
  if (update) lock();
  if (!holding_on_) clearAll();
  std::string name = generate_rgbcloud_name();
  vis_.addPointCloud(pcd, name);
  vis_.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, radius, name);
  rgbcloud_by_handle_[name] = pcd;
  if (update) {
    vis_.spinOnce();
    unlock();
  }
  color_by_handle_[name] = std::vector<int>(3, 0);
  return name;
}
std::string VisWrapper::showCloud(CloudColor_t::ConstPtr pcd, int r, int g,
                                  int b, float radius, bool update) {
  if (update) lock();
  if (!holding_on_) clearAll();
  std::string name = generate_rgbcloud_name();
  CloudColor_t::Ptr pcd_to_add = CloudColor_t::Ptr(new CloudColor_t);
  pcd_to_add->points.resize(pcd->points.size());
  for (size_t i = 0; i < pcd->points.size(); ++i) {
    pcd_to_add->points[i].x = pcd->points[i].x;
    pcd_to_add->points[i].y = pcd->points[i].y;
    pcd_to_add->points[i].z = pcd->points[i].z;
    pcd_to_add->points[i].r = r;
    pcd_to_add->points[i].g = g;
    pcd_to_add->points[i].b = b;
  }
  vis_.addPointCloud(pcd_to_add, name);
  vis_.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, radius, name);
  rgbcloud_by_handle_[name] = pcd_to_add;
  if (update) {
    vis_.spinOnce();
    unlock();
  }
  color_by_handle_[name] = std::vector<int>(3, 0);
  return name;
}
std::string VisWrapper::showCloud(KeyCloud_t::ConstPtr pcd, int r, int g, int b,
                                  float radius, bool update) {
  if (update) lock();
  if (!holding_on_) clearAll();
  pcl::visualization::PointCloudColorHandlerCustom<KeyPoint_t> handler(pcd, r,
                                                                       g, b);
  std::string name = generate_keycloud_name();
  vis_.addPointCloud(pcd, handler, name);
  vis_.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, radius, name);
  keycloud_by_handle_[name] = pcd;
  if (update) {
    vis_.spinOnce();
    unlock();
  }
  color_by_handle_[name] = std::vector<int>(3);
  color_by_handle_[name][0] = r;
  color_by_handle_[name][1] = g;
  color_by_handle_[name][2] = b;
  return name;
}

std::string VisWrapper::showMesh(Mesh_t::ConstPtr mesh, bool update) {
  if (update) lock();
  if (!holding_on_) clearAll();
  std::string name = generate_mesh_name();
  vis_.addPolygonMesh(*mesh, name);
  mesh_by_handle_[name] = mesh;
  if (update) {
    vis_.spinOnce();
    unlock();
  }
  return name;
}

void VisWrapper::updateCloud(const std::string& handle,
                             CloudBW_t::ConstPtr pcd) {
  assert(type_by_handle_[handle] == BWCLOUD_TYPE);
  bwcloud_by_handle_[handle] = pcd;
  vis_.updatePointCloud(pcd, handle);
}
void VisWrapper::updateCloud(const std::string& handle,
                             CloudColor_t::ConstPtr pcd) {
  assert(type_by_handle_[handle] == RGBCLOUD_TYPE);
  rgbcloud_by_handle_[handle] = pcd;
  vis_.updatePointCloud(pcd, handle);
}
void VisWrapper::updateCloud(const std::string& handle,
                             KeyCloud_t::ConstPtr pcd) {
  assert(type_by_handle_[handle] == KEYCLOUD_TYPE);
  keycloud_by_handle_[handle] = pcd;
  vis_.updatePointCloud<KeyPoint_t>(pcd, handle);
}
void VisWrapper::updateMesh(const std::string& handle, Mesh_t::ConstPtr mesh) {
  assert(type_by_handle_[handle] == MESH_TYPE);
  mesh_by_handle_[handle] = mesh;
  // vis_.updatePolygonMesh(mesh, handle);
  vis_.removePolygonMesh(handle);
  vis_.addPolygonMesh(*mesh, handle);
}

std::string VisWrapper::generate_bwcloud_name() {
  std::ostringstream oss;
  oss << "BWCloud_" << rand();
  std::string name = oss.str();
  bwcloud_names_.push_back(name);
  handles_.push_back(name);
  type_by_handle_[name] = BWCLOUD_TYPE;
  return name;
}
std::string VisWrapper::generate_rgbcloud_name() {
  std::ostringstream oss;
  oss << "RGBCloud_" << rand();
  std::string name = oss.str();
  rgbcloud_names_.push_back(name);
  handles_.push_back(name);
  type_by_handle_[name] = RGBCLOUD_TYPE;
  return name;
}
std::string VisWrapper::generate_keycloud_name() {
  std::ostringstream oss;
  oss << "Keycloud_" << rand();
  std::string name = oss.str();
  keycloud_names_.push_back(name);
  handles_.push_back(name);
  type_by_handle_[name] = KEYCLOUD_TYPE;
  return name;
}
std::string VisWrapper::generate_mesh_name() {
  std::ostringstream oss;
  oss << "Mesh_" << rand();
  std::string name = oss.str();
  mesh_names_.push_back(name);
  handles_.push_back(name);
  type_by_handle_[name] = MESH_TYPE;
  return name;
}

void VisWrapper::transform(const Eigen::Affine3f& trans, std::string handle) {
  if (handle == "LAST") handle = handles_[handles_.size() - 1];
  DisplayType type = type_by_handle_[handle];
  switch (type) {
    case BWCLOUD_TYPE: {
      CloudBW_t::Ptr transformed(new CloudBW_t);
      CloudBW_t::ConstPtr cloud = bwcloud_by_handle_[handle];
      pcl::transformPointCloud(*cloud, *transformed, trans);
      pcl::visualization::PointCloudColorHandlerCustom<PointBW_t> handler(
          transformed, color_by_handle_[handle][0], color_by_handle_[handle][1],
          color_by_handle_[handle][2]);
      vis_.updatePointCloud(transformed, handler, handle);
      bwcloud_by_handle_[handle] = transformed;
      break;
    }
    case RGBCLOUD_TYPE: {
      CloudColor_t::Ptr transformed(new CloudColor_t);
      CloudColor_t::ConstPtr cloud = rgbcloud_by_handle_[handle];
      pcl::transformPointCloud(*cloud, *transformed, trans);
      vis_.updatePointCloud(transformed, handle);
      rgbcloud_by_handle_[handle] = transformed;
      break;
    }
    case KEYCLOUD_TYPE: {
      KeyCloud_t::Ptr transformed_key(new KeyCloud_t);
      KeyCloud_t::ConstPtr cloud = keycloud_by_handle_[handle];
      pcl::transformPointCloud(*cloud, *transformed_key, trans);
      pcl::visualization::PointCloudColorHandlerCustom<KeyPoint_t> handler(
          transformed_key, color_by_handle_[handle][0],
          color_by_handle_[handle][1], color_by_handle_[handle][2]);
      vis_.updatePointCloud<KeyPoint_t>(transformed_key, handler, handle);
      keycloud_by_handle_[handle] = transformed_key;
      break;
    }
    case MESH_TYPE: {
      Mesh_t::Ptr transformed_mesh(new Mesh_t);
      Mesh_t::ConstPtr mesh = mesh_by_handle_[handle];
      *transformed_mesh = *mesh;  // Copy
      Cloud_t::Ptr tmp_cloud(new Cloud_t);
      pcl::fromPCLPointCloud2(transformed_mesh->cloud, *tmp_cloud);
      pcl::transformPointCloud(*tmp_cloud, *tmp_cloud, trans);
      pcl::toPCLPointCloud2(*tmp_cloud, transformed_mesh->cloud);
      vis_.removePolygonMesh(handle);
      vis_.addPolygonMesh(*transformed_mesh, handle);
      mesh_by_handle_[handle] = transformed_mesh;
      break;
    }
  }
}
void VisWrapper::transform(float dx, float dy, float dz, std::string handle) {
  Eigen::Affine3f trans;
  trans = Eigen::Translation3f(dx, dy, dz);
  transform(trans, handle);
}

bool VisWrapper::centerCamera(std::string handle) {
  if (handle == "LAST") handle = handles_[handles_.size() - 1];
  if (!type_by_handle_.count(handle)) return false;
  Eigen::Vector4f centroid;
  DisplayType type = type_by_handle_[handle];
  switch (type) {
    case BWCLOUD_TYPE:
      pcl::compute3DCentroid(*(bwcloud_by_handle_[handle]), centroid);
      break;
    case RGBCLOUD_TYPE:
      pcl::compute3DCentroid(*(rgbcloud_by_handle_[handle]), centroid);
      break;
    case KEYCLOUD_TYPE:
      pcl::compute3DCentroid(*(keycloud_by_handle_[handle]), centroid);
      break;
    case MESH_TYPE: {
      Cloud_t cloud;
      pcl::fromPCLPointCloud2(mesh_by_handle_[handle]->cloud, cloud);
      pcl::compute3DCentroid(cloud, centroid);
      break;
    }
  }
  vis_.setCameraPosition(0, 0, 0, centroid(0), centroid(1), centroid(2), 0, -1,
                         0);
}

void VisWrapper::pointPickingCallback(
    const pcl::visualization::PointPickingEvent& event, void* cookie) {
  PointBW_t point;
  event.getPoint(point.x, point.y, point.z);
  // Figure out which cloud it's in
  float thresh = 0.0001f;
  for (size_t i = 0u; i < handles_.size(); ++i) {
    DisplayType type = type_by_handle_[handles_[i]];
    CloudBW_t::ConstPtr cloud;
    switch (type) {
      case BWCLOUD_TYPE:
        cloud = bwcloud_by_handle_[handles_[i]];
        break;
      case RGBCLOUD_TYPE:
        cloud = toCloudBW(rgbcloud_by_handle_[handles_[i]]);
        break;
      case KEYCLOUD_TYPE:
        cloud = toCloudBW(keycloud_by_handle_[handles_[i]]);
        break;
      case MESH_TYPE:
        cloud = toCloudBW(mesh_by_handle_[handles_[i]]);
        break;
    }
    pcl::search::KdTree<PointBW_t> tree;
    tree.setInputCloud(cloud);
    std::vector<float> dists;
    std::vector<int> idxs;
    if (tree.nearestKSearch(point, 1, idxs, dists)) {
      if (fabs(dists[0]) < thresh) {
        ppc_handle_ = handles_[i];
        ppc_idx_ = idxs[0];
        ppc_updated_ = true;
        ppc_point_ = point;
        return pointPickingCallbackImpl(point, handles_[i], idxs[0]);
      }
    }
  }
}

CloudBW_t::ConstPtr VisWrapper::toCloudBW(
    const CloudColor_t::ConstPtr& cloud) const {
  CloudBW_t::Ptr bw(new CloudBW_t);
  bw->height = cloud->height;
  bw->width = cloud->width;
  bw->points.resize(cloud->points.size());
  for (size_t i = 0; i < cloud->points.size(); ++i) {
    bw->points[i].x = cloud->points[i].x;
    bw->points[i].y = cloud->points[i].y;
    bw->points[i].z = cloud->points[i].z;
  }
  return (CloudBW_t::ConstPtr)bw;
}
CloudBW_t::ConstPtr VisWrapper::toCloudBW(
    const KeyCloud_t::ConstPtr& cloud) const {
  CloudBW_t::Ptr bw(new CloudBW_t);
  bw->height = cloud->height;
  bw->width = cloud->width;
  bw->points.resize(cloud->points.size());
  for (size_t i = 0u; i < cloud->points.size(); ++i) {
    bw->points[i].x = cloud->points[i].x;
    bw->points[i].y = cloud->points[i].y;
    bw->points[i].z = cloud->points[i].z;
  }
  return (CloudBW_t::ConstPtr)bw;
}
CloudBW_t::ConstPtr VisWrapper::toCloudBW(const Mesh_t::ConstPtr& mesh) const {
  CloudBW_t::Ptr bw(new CloudBW_t);
  pcl::fromPCLPointCloud2(mesh->cloud, *bw);
  return (CloudBW_t::ConstPtr)bw;
}
