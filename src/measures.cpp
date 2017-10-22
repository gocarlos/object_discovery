#include "object_discovery/measures.h"

#include <vector>

#include <pcl/conversions.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/impl/convex_hull.hpp>

#include "object_discovery/geom_utils.h"
#include "object_discovery/utils.h"

namespace object_discovery {

float Measures::scoreCompactness(const Cloud_t::Ptr cloud) {
  CHECK_NOTNULL(cloud.get());
  CHECK_GT(cloud->points.size(), 0);
  // This implementation maximizes speed to readability ratio.
  float maxx = cloud->points[0].x;
  float minx = cloud->points[0].x;
  for (size_t i = 1u; i < cloud->points.size(); ++i) {
    float x = cloud->points[i].x;
    if (x > maxx) maxx = x;
    if (x < minx) minx = x;
  }
  return cloud->points.size() / pow(maxx - minx, 2);
}

float Measures::scoreSymmetry(const Cloud_t::Ptr cloud,
                              const NormalCloud_t::Ptr normals,
                              const float relweight) {
  CHECK_NOTNULL(cloud.get());
  CHECK_NOTNULL(normals.get());
  CHECK_GT(cloud->points.size(), 0);
  CHECK_GT(normals->points.size(), 0);

  Eigen::MatrixXf ptsmap = cloud->getMatrixXfMap();
  Eigen::VectorXf mins = ptsmap.rowwise().minCoeff();
  Eigen::VectorXf maxes = ptsmap.rowwise().maxCoeff();
  Eigen::VectorXf ranges = maxes - mins;
  // x,y,z are always the first 3 dimensions
  Eigen::VectorXf w = ranges.head(3);
  w /= w.sum();

  float score = 0.0f;
  Cloud_t::Ptr dest(new Cloud_t);
  dest->points.resize(cloud->points.size());
  NormalCloud_t::Ptr normdest(new NormalCloud_t);
  normdest->points.resize(normals->points.size());
  for (size_t t = 0u; t < 3; ++t) {
    for (size_t i = 0u; i < cloud->points.size(); ++i) {
      Point_t& d = dest->points[i];
      Point_t& c = cloud->points[i];
      Normal_t& dn = normdest->points[i];
      Normal_t& cn = normals->points[i];
      if (t == 0) {
        d.x = -c.x;
        dn.normal_x = -cn.normal_x;
      } else {
        d.x = c.x;
        dn.normal_x = cn.normal_x;
      }
      if (t == 1) {
        d.y = -c.y;
        dn.normal_y = -cn.normal_y;
      } else {
        d.y = c.y;
        dn.normal_y = cn.normal_y;
      }
      if (t == 2) {
        d.z = -c.z;
        dn.normal_z = -cn.normal_z;
      } else {
        d.z = c.z;
        dn.normal_y = cn.normal_y;
      }
    }

    float dnormalize = ranges(t);
    float overlap =
        getOverlap(cloud, normals, dest, normdest, relweight, dnormalize);

    score += w(t) * overlap;
  }
  // Return the average symmetry.
  return -score;
}

float Measures::getOverlap(const Cloud_t::Ptr cloud,
                           const NormalCloud_t::Ptr normals, Cloud_t::Ptr dest,
                           NormalCloud_t::Ptr normdest, const float relweight,
                           float dnormalize) {
  float overlap = geom_utils_.cloudAlignmentScoreDenseWithNormalsNormalized(
                      cloud, normals, dest, normdest, relweight, dnormalize) +
                  geom_utils_.cloudAlignmentScoreDenseWithNormalsNormalized(
                      dest, normdest, cloud, normals, relweight, dnormalize);
  return overlap;
}

float Measures::scoreGlobalConvexity(const Cloud_t::Ptr cloud) {
  CHECK_NOTNULL(cloud.get());
  CHECK_GT(cloud->points.size(), 0);

  pcl::ConvexHull<Point_t> chull;
  chull.setInputCloud(cloud);
  chull.setDimension(3);
  std::vector<pcl::Vertices> polygons;
  Cloud_t::Ptr hull(new Cloud_t);
  chull.reconstruct(*hull, polygons);
  CHECK_GT(polygons.size(), 0);

  // Make a full Mesh_t out of it.
  Mesh_t::Ptr m(new Mesh_t);
  m->polygons = polygons;
  pcl::toPCLPointCloud2(*hull, m->cloud);

  // Subdivide it:  0.01 is not really a free parameter. The smaller the better
  // and 1.5cm is reasonable.
  constexpr float threshold_to_subdivide = 0.015f;
  Mesh_t::Ptr msub =
      geom_utils_.meshSubdivideRecursive(m, threshold_to_subdivide);
  Cloud_t::Ptr verts(new Cloud_t);
  pcl::fromPCLPointCloud2(msub->cloud, *verts);

  // Calculate alignment from object to hull.
  float overlap = geom_utils_.cloudAlignmentScoreDense(cloud, verts);
  return -overlap;
}

float Measures::scoreLocalConvexity(const Cloud_t::Ptr cloud,
                                    const NormalCloud_t::Ptr ncloud,
                                    const float NNradius) {
  CHECK_NOTNULL(cloud.get());
  CHECK_NOTNULL(ncloud.get());
  CHECK_GT(cloud->points.size(), 0);

  Tree_t nntree;
  nntree.setInputCloud(cloud);
  size_t number_of_points = cloud->points.size();
  std::vector<int> k_indices;
  std::vector<float> k_sqr_distances;
  float score = 0.0f;
  for (size_t i = 0u; i < number_of_points; ++i) {
    nntree.radiusSearch(i, NNradius, k_indices, k_sqr_distances);
    Normal_t& first_normal = ncloud->points[i];
    Point_t& first_cloud_point = cloud->points[i];
    float dots = 0.0f;
    size_t nc = 0u;
    for (size_t j = 0u; j < k_indices.size(); ++j) {
      if (k_indices[j] == i) {
        continue;
      }
      Normal_t& second_normal = ncloud->points[k_indices[j]];
      Point_t& second_cloud_point = cloud->points[k_indices[j]];

      float dot = first_normal.normal_x * second_normal.normal_x +
                  first_normal.normal_y * second_normal.normal_y +
                  first_normal.normal_z * second_normal.normal_z;

      float a = second_cloud_point.x - first_cloud_point.x;
      float b = second_cloud_point.y - first_cloud_point.y;
      float c = second_cloud_point.z - first_cloud_point.z;
      float d = sqrt(a * a + b * b + c * c);
      a /= d;
      b /= d;
      c /= d;
      float dot2 = second_normal.normal_x * a + second_normal.normal_y * b +
                   second_normal.normal_z * c;

      if (dot2 > 0.0f) {
        nc++;
      }
    }
    score += (1.0f * nc) / k_indices.size();
  }
  return score / number_of_points;
}

float Measures::scoreSmoothness(const Cloud_t::Ptr cloud,
                                const NormalCloud_t::Ptr ncloud,
                                const float NNradius, int num_bins) {
  Tree_t nntree;
  nntree.setInputCloud(cloud);
  size_t number_of_cloud_points = cloud->points.size();
  std::vector<int> ix;
  std::vector<float> dd;
  float Hs = 0.0f;

  for (size_t i = 0u; i < number_of_cloud_points; ++i) {
    // Find a vector perpendicular to n1 by orthogonalizing.
    Normal_t& n1 = ncloud->points[i];
    Normal_t n2(utils_.randf(-1, 1), utils_.randf(-1, 1), utils_.randf(-1, 1));
    float dot = n1.normal_x * n2.normal_x + n1.normal_y * n2.normal_y +
                n1.normal_z * n2.normal_z;
    // Make sure that the normals don't have normal components.
    CHECK(!std::isnan(n1.normal_x) && !std::isnan(n1.normal_y) &&
          !std::isnan(n1.normal_z));

    n2.normal_x = n2.normal_x - dot * n1.normal_x;
    n2.normal_y = n2.normal_y - dot * n1.normal_y;
    n2.normal_z = n2.normal_z - dot * n1.normal_z;

    // Normalize the orthogonal vector.
    float ddd =
        std::sqrt(n2.normal_x * n2.normal_x + n2.normal_y * n2.normal_y +
                  n2.normal_z * n2.normal_z);
    n2.normal_x /= ddd;
    n2.normal_y /= ddd;
    n2.normal_z /= ddd;

    // Find the second vector using cross product.
    Normal_t n3;
    n3.normal_x = n1.normal_y * n2.normal_z - n1.normal_z * n2.normal_y;
    n3.normal_y = n1.normal_z * n2.normal_x - n1.normal_x * n2.normal_z;
    n3.normal_z = n1.normal_x * n2.normal_y - n1.normal_y * n2.normal_x;

    Point_t& p1 = cloud->points[i];
    std::vector<float> bins(num_bins, 1.0f);
    float binsum = num_bins;
    nntree.radiusSearch(i, NNradius, ix, dd);
    for (size_t j = 0u; j < ix.size(); ++j) {
      if (ix[j] == i) {
        continue;
      }
      Point_t& p2 = cloud->points[ix[j]];
      if (p2.x - p1.x == 0 && p2.y - p1.y == 0 && p2.z - p1.z == 0) {
        printf("POINTS TOO CLOSE !\n");
        continue;
      }
      float a = p2.x - p1.x;
      float b = p2.y - p1.y;
      float c = p2.z - p1.z;
      float d = sqrt(a * a + b * b + c * c);
      a /= d;
      b /= d;
      c /= d;

      float dot1 = n2.normal_x * a + n2.normal_y * b + n2.normal_z * c;
      float dot2 = n3.normal_x * a + n3.normal_y * b + n3.normal_z * c;

      // Angle in range 0->1
      float theta = (atan2(dot1, dot2) + M_PI) / 2.0 / M_PI;

      // Subbing 0.001 is an ugly hack but that's ok. It prevents
      // bin_id=num_bins.
      int bin_id = static_cast<size_t>((theta - 0.001f) * num_bins);
      bins[bin_id] += 1.0f;
      binsum += 1.0f;
    }

    float H = 0.0f;
    for (size_t j = 0u; j < num_bins; ++j) {
      bins[j] /= binsum;
    }
    for (size_t j = 0u; j < num_bins; ++j) {
      H -= bins[j] * log(bins[j]);
    }
    Hs += H;
  }
  // High entropy is good.
  return Hs / number_of_cloud_points;
}
}  // namespace object_discovery
