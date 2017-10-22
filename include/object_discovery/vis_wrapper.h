#pragma once

#include <map>
#include <string>
#include <vector>

#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "object_discovery/lockable.h"
#include "object_discovery/utils.h"

class VisWrapper : public Lockable {
 public:
  enum DisplayType { BWCLOUD_TYPE, RGBCLOUD_TYPE, KEYCLOUD_TYPE, MESH_TYPE };

  pcl::visualization::PCLVisualizer vis_;
  typedef boost::shared_ptr<VisWrapper> Ptr;
  typedef boost::shared_ptr<const VisWrapper> ConstPtr;

  VisWrapper(std::string name = "VisWrapper", bool pick_points = false,
             bool mouse_cb = false);

  char waitKey(int msec = 0);

  void close();

  PointBW_t waitPoint(std::string& handle, int& idx);
  PointBW_t waitPoint();
  void waitLoop(char exitKey);
  // Return handles
  std::string showCloud(CloudBW_t::ConstPtr pcd, int r = 255, int g = 255,
                        int b = 255, float radius = 1, bool update = true);
  std::string showCloud(CloudColor_t::ConstPtr pcd, float radius = 1,
                        bool update = true);
  std::string showCloud(CloudColor_t::ConstPtr pcd, int r, int g, int b,
                        float radius = 1, bool update = true);
  std::string showCloud(KeyCloud_t::ConstPtr pcd, int r = 0, int g = 0,
                        int b = 255, float radius = 3, bool update = true);
  std::string showMesh(Mesh_t::ConstPtr mesh, bool update = true);
  /*  Update rather than show */
  void updateCloud(const std::string& handle, CloudBW_t::ConstPtr pcd);
  void updateCloud(const std::string& handle, CloudColor_t::ConstPtr pcd);
  void updateCloud(const std::string& handle, KeyCloud_t::ConstPtr pcd);
  void updateMesh(const std::string& handle, Mesh_t::ConstPtr mesh);
  /* End return handles */
  void transform(const Eigen::Affine3f& trans, std::string handle = "LAST");
  void transform(float dx, float dy, float dz, std::string handle = "LAST");
  void holdOn();
  void holdOff();
  void remove(std::string handle);
  void clearAll();
  bool centerCamera(std::string handle = "LAST");

 protected:
  char key_;
  std::string name_;
  bool mouse_cb_;
  void keyboardCallback(const pcl::visualization::KeyboardEvent& event,
                        void* cookie);
  void mouseCallback(const pcl::visualization::MouseEvent& event, void* cookie);
  bool holding_on_;
  std::vector<std::string> bwcloud_names_, rgbcloud_names_, keycloud_names_,
      mesh_names_;

  std::string generate_bwcloud_name();
  std::string generate_rgbcloud_name();
  std::string generate_keycloud_name();
  std::string generate_mesh_name();

  CloudBW_t::ConstPtr toCloudBW(const CloudColor_t::ConstPtr& cloud) const;
  CloudBW_t::ConstPtr toCloudBW(const KeyCloud_t::ConstPtr& cloud) const;
  CloudBW_t::ConstPtr toCloudBW(const Mesh_t::ConstPtr& mesh) const;

  std::vector<std::string> handles_;
  std::map<std::string, DisplayType> type_by_handle_;
  std::map<std::string, CloudBW_t::ConstPtr> bwcloud_by_handle_;
  std::map<std::string, CloudColor_t::ConstPtr> rgbcloud_by_handle_;
  std::map<std::string, KeyCloud_t::ConstPtr> keycloud_by_handle_;
  std::map<std::string, Mesh_t::ConstPtr> mesh_by_handle_;
  std::map<std::string, std::vector<int>> color_by_handle_;

  // Point picking things
  bool ppc_updated_;
  std::string ppc_handle_;
  int ppc_idx_;
  PointBW_t ppc_point_;

  void pointPickingCallback(const pcl::visualization::PointPickingEvent& event,
                            void* cookie);
  virtual void pointPickingCallbackImpl(const PointBW_t& point,
                                        const std::string& handle, int idx) {}
  virtual void keyboardCallbackImpl(char key) {}
  virtual void mouseCallbackImpl(int x, int y,
                                 const pcl::visualization::MouseEvent& event) {}
  virtual void runEveryWaitKey() {}
};
