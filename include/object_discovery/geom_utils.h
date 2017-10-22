#pragma once

#include <glog/logging.h>
#include <Eigen/Dense>
#include <vector>
// PCL
#include <pcl/common/pca.h>
#include <pcl/features/moment_of_inertia_estimation.h>  // Center of point cloud
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/ros/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

// ROS
#include <sensor_msgs/point_cloud2_iterator.h>
#include <shape_msgs/Mesh.h>

#include "object_discovery/vis_wrapper.h"

namespace object_discovery {
class GeomUtils {
 public:
  GeomUtils();

  /*!
   * Calculates for a given mesh the point cloud and the point normal. The point
   * normal are smoothified, that means that each vertices votes for the normal
   * of its neighbors.
   * param[in] mesh_in
   * param[out] cloud_out
   * param[out] normals_out
   */
  void convertMeshToPointsNormals(const Mesh_t::ConstPtr& mesh_in,
                                  Cloud_t::Ptr& cloud_out,
                                  NormalCloud_t::Ptr& normals_out);

  void convertPolygonMeshToRosMesh(const pcl::PolygonMesh::Ptr& pcl_mesh,
                                   shape_msgs::Mesh::Ptr ros_mesh);

  //! Return all edges, WITH duplicates
  void getEdges(const Mesh_t::Ptr mesh, std::vector<std::pair<int, int>>& e);

  void showCloudsGrid(VisWrapper& vis_wrapper,
                      std::vector<Cloud_t::Ptr>& clouds);

  void showCloudsGridOriented(VisWrapper& vis_wrapper,
                              std::vector<Cloud_t::Ptr>& clouds,
                              std::vector<NormalCloud_t::Ptr>& normals);

  /*!
   * Method which takes point cloud, projects them to the eigenbases
   * and returns the projected clouds as well as the eigenvalues.
   * @param[in] cloud PCL point cloud
   * @param[in] normal PCL point normal
   * @param[out] projcloud PCL point cloud
   * @param[out] projnormals PCL point normal
   * @param[out] lambda1normalized second eigenvalue of scatter matrix divided
   * by the first
   * @param[out] lambda2normalized third eigenvalue of scatter matrix divided
   * by the first
   * (lambda2 < lambda1)
   */
  void projectToEigenbasis(const Cloud_t::Ptr cloud,
                           const NormalCloud_t::Ptr normals,
                           Cloud_t::Ptr& projcloud,
                           NormalCloud_t::Ptr& projnormals,
                           float& lambda1normalized, float& lambda2normalized);

  float cloudAlignmentScoreDenseWithNormalsNormalized(
      const Cloud_t::Ptr& cloud1, const NormalCloud_t::Ptr& ncloud1,
      const Cloud_t::Ptr& cloud2, const NormalCloud_t::Ptr& ncloud2,
      float relweight, float dnormalize);

  /*!
   * Calculate the mean distance between points in both given clouds.
   * @param[in] cloud1 input PCL point cloud
   * @param[in] cloud2 input PCL point cloud
   * @return mean of distances from points in cloud1 to their nearest
   * neighbors in cloud2
   */
  float cloudAlignmentScoreDense(const Cloud_t::Ptr& cloud1,
                                 const Cloud_t::Ptr& cloud2);

  float blendColors(float rgb1f, float rgb2f);
  float blendColors(float rgb1f, float rgb2f, float rgb3f);

  Mesh_t::Ptr meshSubdivide(const Mesh_t::ConstPtr& mesh, float thr);

  Mesh_t::Ptr meshSubdivideRecursive(const Mesh_t::ConstPtr& mesh, float thr);

  /*!
   * Calculates the normals at each point on the surface.
   * @param[in] xyz_cloud_in
   * @param[out] point_normal_cloud_out
   */
  void calculateNormals(
      const pcl::PointCloud<Point_t>::Ptr xyz_cloud_in,
      pcl::PointCloud<pcl::PointNormal>::Ptr point_normal_cloud_out);

  /*!
   * Method to calculate the principle axis of a given point cloud
   * as well as the center of mass and bounding box.
   * @param[in] cloud_in
   * @param[out] position_out
   * @param[out] orientation_out
   * @param[out] scale_out
   */
  void calculatePCA(const pcl::PointCloud<Point_t>::Ptr cloud_in,
                    Eigen::Vector3f& position_out,
                    Eigen::Quaternionf& orientation_out,
                    Eigen::Vector3f& scale_out,
                    Eigen::Vector3f& mass_center_out);

  struct RGBStruct {
    RGBStruct(float _rgb) { rgb = _rgb; }
    RGBStruct() {
      s.r = 0;
      s.g = 0;
      s.b = 0;
      s.alpha = 0;
    }
    union {
      float rgb;
      struct {
        uint8_t b;
        uint8_t g;
        uint8_t r;
        uint8_t alpha;
      } s;
    };
  };

  //! Parameters for PointNormal estimation.
  double radius_search_;
  bool set_polynomial_fit_;
  bool set_compute_normals_;
};

}  // namespace object_discovery
