
#include "object_discovery/geom_utils.h"

namespace object_discovery {

GeomUtils::GeomUtils()
    : radius_search_(0.01f),
      set_polynomial_fit_(true),
      set_compute_normals_(true) {}

void GeomUtils::convertMeshToPointsNormals(const Mesh_t::ConstPtr& mesh,
                                           Cloud_t::Ptr& cloud,
                                           NormalCloud_t::Ptr& normals) {
  std::cout << "Get normals from PCL PolygonMesh." << std::endl;

  // Unfold mesh, which is stored as ROS message.
  pcl::fromPCLPointCloud2(mesh->cloud, *cloud);
  std::vector<int> counts(cloud->points.size(), 0);
  normals->points.resize(cloud->points.size());

  for (size_t i = 0u; i < mesh->polygons.size(); i++) {
    const pcl::Vertices& vv = mesh->polygons[i];

    // Get the 3 points.
    int i1 = vv.vertices[0];
    int i2 = vv.vertices[1];
    int i3 = vv.vertices[2];
    Point_t& p1 = cloud->points[i1];
    Point_t& p2 = cloud->points[i2];
    Point_t& p3 = cloud->points[i3];

    // Convert to eigen points.
    Eigen::Vector3d pe1(p1.x, p1.y, p1.z);
    Eigen::Vector3d pe2(p2.x, p2.y, p2.z);
    Eigen::Vector3d pe3(p3.x, p3.y, p3.z);

    // Find normal.
    Eigen::Vector3d normal = (pe2 - pe1).cross(pe3 - pe1);
    normal = normal / normal.norm();
    Normal_t pnormal(normal[0], normal[1], normal[2]);

    // Smoothly blend with the old normal estimate at this point. Basically each
    // face votes for the normal of the verteces around it.
    float v;
    Normal_t a;
    a = normals->points[i1];
    v = 1.0 / (counts[i1] + 1.0);
    normals->points[i1] =
        Normal_t(v * pnormal.normal_x + (1.0 - v) * a.normal_x,
                 v * pnormal.normal_y + (1.0 - v) * a.normal_y,
                 v * pnormal.normal_z + (1.0 - v) * a.normal_z);

    a = normals->points[i2];
    v = 1.0 / (counts[i2] + 1.0);
    normals->points[i2] =
        Normal_t(v * pnormal.normal_x + (1.0 - v) * a.normal_x,
                 v * pnormal.normal_y + (1.0 - v) * a.normal_y,
                 v * pnormal.normal_z + (1.0 - v) * a.normal_z);

    a = normals->points[i3];
    v = 1.0 / (counts[i3] + 1.0);
    normals->points[i3] =
        Normal_t(v * pnormal.normal_x + (1.0 - v) * a.normal_x,
                 v * pnormal.normal_y + (1.0 - v) * a.normal_y,
                 v * pnormal.normal_z + (1.0 - v) * a.normal_z);

    counts[i1]++;
    counts[i2]++;
    counts[i3]++;
  }
}

// Return all edges, WITH duplicates
void GeomUtils::getEdges(const Mesh_t::Ptr mesh,
                         std::vector<std::pair<int, int>>& e) {
  for (size_t i = 0u; i < mesh->polygons.size(); ++i) {
    const pcl::Vertices& v = mesh->polygons[i];
    int v1 = v.vertices[0];
    int v2 = v.vertices[1];
    int v3 = v.vertices[2];
    e.push_back(std::pair<int, int>(v1, v2));
    e.push_back(std::pair<int, int>(v1, v3));
    e.push_back(std::pair<int, int>(v3, v2));
  }
}

void GeomUtils::showCloudsGrid(VisWrapper& vis_wrapper,
                               std::vector<Cloud_t::Ptr>& clouds) {
  constexpr float space = 0.5f;
  int gsize = (int)sqrt(clouds.size());
  Eigen::Affine3f trans;
  vis_wrapper.holdOff();
  for (size_t i = 0u; i < clouds.size(); ++i) {
    Cloud_t::Ptr cdisp(new Cloud_t(*(clouds[i])));
    trans = Eigen::Translation3f((i / gsize) * space, (i % gsize) * space, 0);
    pcl::transformPointCloud(*(cdisp), *(cdisp), trans);
    vis_wrapper.showCloud(cdisp, 0.02f, false);
    vis_wrapper.holdOn();
  }
  vis_wrapper.vis_.spinOnce();
}

void GeomUtils::showCloudsGridOriented(
    VisWrapper& vis_wrapper, std::vector<Cloud_t::Ptr>& clouds,
    std::vector<NormalCloud_t::Ptr>& normals) {
  constexpr float space = 0.5;
  int gsize = (int)sqrt(clouds.size());
  Eigen::Affine3f trans;
  vis_wrapper.holdOff();
  for (size_t i = 0u; i < clouds.size(); ++i) {
    Cloud_t::Ptr current_display_point_cloud(new Cloud_t(*(clouds[i])));

    // Make the majority of the normals point to the view point.
    float z_mass = 0;
    for (size_t k = 0u; k < current_display_point_cloud->points.size(); ++k) {
      Normal_t& n = normals[i]->points[k];
      z_mass += n.normal_z;
    }
    if (z_mass > 0) {
      // Flip all the normals, if the average of the z-normal-component is
      // pointing away from the viewpoint.
      for (int k = 0; k < current_display_point_cloud->points.size(); k++) {
        current_display_point_cloud->points[k].z =
            -current_display_point_cloud->points[k].z;
      }
    }

    trans = Eigen::Translation3f((i / gsize) * space, (i % gsize) * space, 0);
    pcl::transformPointCloud(*(current_display_point_cloud),
                             *(current_display_point_cloud), trans);
    vis_wrapper.showCloud(current_display_point_cloud, 0.02f, false);
    vis_wrapper.holdOn();
  }
  vis_wrapper.vis_.spinOnce();
}

void GeomUtils::projectToEigenbasis(const Cloud_t::Ptr cloud,
                                    const NormalCloud_t::Ptr normals,
                                    Cloud_t::Ptr& projcloud,
                                    NormalCloud_t::Ptr& projnormals,
                                    float& lambda1normalized,
                                    float& lambda2normalized) {
  pcl::PCA<Point_t> pca;
  pca.setInputCloud(cloud);
  Eigen::Matrix3f eigvecs = pca.getEigenVectors();
  float det = pcl::determinant3x3Matrix(eigvecs);
  if (det < 0) {
    // Guarantee a rotation. sigh PCL!
    eigvecs = -eigvecs;
  }

  Eigen::Vector4f segmean;
  segmean = Eigen::Vector4f::Zero();
  pcl::compute3DCentroid(*cloud, segmean);
  int number_of_cloud_points = cloud->points.size();
  Cloud_t::Ptr proj(new Cloud_t);
  NormalCloud_t::Ptr nproj(new NormalCloud_t);
  proj->points.resize(number_of_cloud_points);
  nproj->points.resize(number_of_cloud_points);
  for (size_t j = 0u; j < number_of_cloud_points; ++j) {
    // Project points (after subbing the mean).
    Eigen::Vector3f di = cloud->points[j].getVector3fMap() - segmean.head<3>();
    proj->points[j].getVector3fMap() = eigvecs.transpose() * di;

    // Project the normals.
    nproj->points[j].getNormalVector3fMap() =
        eigvecs.transpose() * normals->points[j].getNormalVector3fMap();
  }
  proj->width = 1u;
  proj->height = number_of_cloud_points;
  nproj->width = 1u;
  nproj->height = number_of_cloud_points;

  // We have to do funny business because PCA doesn't preserve color of
  // points, also we don't just copy over RGB in case we don't want to use
  // color in future some point.
  Cloud_t::Ptr newcloud(new Cloud_t(*cloud));
  for (size_t j = 0u; j < proj->points.size(); ++j) {
    Point_t& p1 = newcloud->points[j];
    Point_t& p2 = proj->points[j];
    p1.x = p2.x;
    p1.y = p2.y;
    p1.z = p2.z;
  }
  newcloud->width = 1u;
  newcloud->height = proj->points.size();

  Eigen::Vector3f eigvals = pca.getEigenValues();
  eigvals /= eigvals.maxCoeff();

  // Outputs
  lambda1normalized = eigvals(1);
  lambda2normalized = eigvals(2);
  projcloud = newcloud;
  projnormals = nproj;
}

float GeomUtils::cloudAlignmentScoreDenseWithNormalsNormalized(
    const Cloud_t::Ptr& cloud1, const NormalCloud_t::Ptr& ncloud1,
    const Cloud_t::Ptr& cloud2, const NormalCloud_t::Ptr& ncloud2,
    float relweight, float dnormalize) {
  std::vector<int> k_indices;
  std::vector<float> k_sqr_distances;
  Tree_t nntree;
  nntree.setInputCloud(cloud2);
  int number_of_points = cloud1->points.size();
  float accum = 0.0f;
  for (size_t i = 0u; i < number_of_points; ++i) {
    nntree.nearestKSearch(cloud1->points[i], 1, k_indices, k_sqr_distances);

    // Normalize by provided length (usually extent of object).
    accum += sqrt(k_sqr_distances[0]) / dnormalize;

    // Also compare normals.
    float dot = ncloud1->points[i].getNormalVector3fMap().dot(
        ncloud2->points[k_indices[0]].getNormalVector3fMap());
    accum += relweight * (1.0f - dot);
  }
  return accum / number_of_points;
}

float GeomUtils::cloudAlignmentScoreDense(const Cloud_t::Ptr& cloud1,
                                          const Cloud_t::Ptr& cloud2) {
  std::vector<int> ix;
  std::vector<float> dd;
  Tree_t nntree;
  nntree.setInputCloud(cloud2);
  int number_of_cloud_points = cloud1->points.size();
  float accum = 0.0f;
  for (size_t i = 0u; i < number_of_cloud_points; ++i) {
    nntree.nearestKSearch(cloud1->points[i], 1, ix, dd);
    accum += sqrt(dd[0]);
  }
  return accum / number_of_cloud_points;
}

float GeomUtils::blendColors(float rgb1f, float rgb2f) {
  RGBStruct rgb1(rgb1f);
  RGBStruct rgb2(rgb2f);
  RGBStruct rgb_out;
  rgb_out.s.r = ((int)rgb1.s.r + (int)rgb2.s.r) / 2;
  rgb_out.s.g = ((int)rgb1.s.g + (int)rgb2.s.g) / 2;
  rgb_out.s.b = ((int)rgb1.s.b + (int)rgb2.s.b) / 2;
  return rgb_out.rgb;
}
float GeomUtils::blendColors(float rgb1f, float rgb2f, float rgb3f) {
  RGBStruct rgb1(rgb1f), rgb2(rgb2f), rgb3(rgb3f);
  RGBStruct rgb_out;
  rgb_out.s.r = ((int)rgb1.s.r + (int)rgb2.s.r + (int)rgb3.s.r) / 3;
  rgb_out.s.g = ((int)rgb1.s.g + (int)rgb2.s.g + (int)rgb3.s.g) / 3;
  rgb_out.s.b = ((int)rgb1.s.b + (int)rgb2.s.b + (int)rgb3.s.b) / 3;
  return rgb_out.rgb;
}

Mesh_t::Ptr GeomUtils::meshSubdivide(const Mesh_t::ConstPtr& mesh_in,
                                     float thr) {
  // Make a replicate for our new mesh.
  Mesh_t::Ptr mesh_out(new Mesh_t);
  *mesh_out = *mesh_in;

  // Unfold mesh, which is stored as ROS message.
  Cloud_t::Ptr pcl_cloud(new Cloud_t);
  pcl::fromPCLPointCloud2(mesh_in->cloud, *pcl_cloud);
  std::vector<pcl::Vertices> toadd;
  for (size_t i = 0; i < mesh_in->polygons.size(); i++) {
    const pcl::Vertices& v = mesh_in->polygons[i];

    // Get the 3 points.
    Point_t& p1 = pcl_cloud->points[v.vertices[0]];
    Point_t& p2 = pcl_cloud->points[v.vertices[1]];
    Point_t& p3 = pcl_cloud->points[v.vertices[2]];

    // Convert to eigen points.
    Eigen::Vector3d pe1(p1.x, p1.y, p1.z);
    Eigen::Vector3d pe2(p2.x, p2.y, p2.z);
    Eigen::Vector3d pe3(p3.x, p3.y, p3.z);

    // Find length of maximal edge.
    float l1 = (pe1 - pe2).norm();
    float l2 = (pe2 - pe3).norm();
    float l3 = (pe3 - pe1).norm();
    float maxedge = std::max(std::max(l1, l2), l3);
    // Below threshold, let's not subdivide.
    if (maxedge < thr) {
      continue;
    }

    // Find edge points of the polygon.
    Eigen::Vector3d c1 = (pe1 + pe2) / 2;
    Eigen::Vector3d c2 = (pe2 + pe3) / 2;
    Eigen::Vector3d c3 = (pe1 + pe3) / 2;

    // Subdivide this polygon.
    pcl::Vertices pnew;
    pnew.vertices.resize(3);

    int dontadd = -1;
    size_t n1, n2;
    // Find indices of longest 2 edges.
    if (l3 <= l1 && l3 <= l2) {
      Point_t pt1;
      pt1.x = c1[0];
      pt1.y = c1[1];
      pt1.z = c1[2];
      Point_t pt2;
      pt2.x = c2[0];
      pt2.y = c2[1];
      pt2.z = c2[2];
#ifdef USE_COLOR
      pt1.rgb = blendColors(p1.rgb, p2.rgb);
      pt2.rgb = blendColors(p2.rgb, p3.rgb);
#endif
      pcl_cloud->points.push_back(pt1);
      pcl_cloud->points.push_back(pt2);
      n1 = pcl_cloud->points.size() - 2;
      n2 = pcl_cloud->points.size() - 1;

      pnew.vertices[0] = v.vertices[0];
      pnew.vertices[1] = v.vertices[2];
      pnew.vertices[2] = n2;

      dontadd = 2;
    } else if (l2 <= l1 && l2 <= l3) {
      Point_t pt1;
      pt1.x = c3[0];
      pt1.y = c3[1];
      pt1.z = c3[2];
      Point_t pt2;
      pt2.x = c1[0];
      pt2.y = c1[1];
      pt2.z = c1[2];
#ifdef USE_COLOR
      pt1.rgb = blendColors(p1.rgb, p3.rgb);
      pt2.rgb = blendColors(p1.rgb, p2.rgb);
#endif
      pcl_cloud->points.push_back(pt1);
      pcl_cloud->points.push_back(pt2);
      n1 = pcl_cloud->points.size() - 2;
      n2 = pcl_cloud->points.size() - 1;

      pnew.vertices[0] = v.vertices[2];
      pnew.vertices[1] = v.vertices[1];
      pnew.vertices[2] = n2;

      dontadd = 1;
    } else {  // if(l1<=l2 && l1<=l3) {
      Point_t pt1;
      pt1.x = c2[0];
      pt1.y = c2[1];
      pt1.z = c2[2];
      Point_t pt2;
      pt2.x = c3[0];
      pt2.y = c3[1];
      pt2.z = c3[2];
#ifdef USE_COLOR
      pt1.rgb = blendColors(p2.rgb, p3.rgb);
      pt2.rgb = blendColors(p1.rgb, p3.rgb);
#endif
      pcl_cloud->points.push_back(pt1);
      pcl_cloud->points.push_back(pt2);
      n1 = pcl_cloud->points.size() - 2;
      n2 = pcl_cloud->points.size() - 1;

      pnew.vertices[0] = v.vertices[1];
      pnew.vertices[1] = v.vertices[0];
      pnew.vertices[2] = n2;

      dontadd = 0;
    }

    // Overwrite one polygon.
    mesh_out->polygons[i] = pnew;

    // Add other polygons to our toadd vector.
    for (size_t j = 0u; j < 3; ++j) {
      if (j == dontadd) continue;
      pnew.vertices[0] = v.vertices[j];
      pnew.vertices[1] = n1;
      pnew.vertices[2] = n2;
      toadd.push_back(pnew);
    }
  }

  for (size_t k = 0u; k < toadd.size(); ++k) {
    mesh_out->polygons.push_back(toadd[k]);
  }

  // Also copy over the new cloud of points to new mesh.
  pcl_cloud->width = pcl_cloud->points.size();
  pcl_cloud->height = 1u;
  pcl::toPCLPointCloud2(*pcl_cloud, mesh_out->cloud);
  return mesh_out;
}

Mesh_t::Ptr GeomUtils::meshSubdivideRecursive(const Mesh_t::ConstPtr& mesh,
                                              float thr) {
  CHECK_NOTNULL(mesh.get());
  CHECK_GT(mesh->polygons.size(), 0);

  Mesh_t::Ptr mesh2(new Mesh_t);
  *mesh2 = *mesh;
  while (true) {
    int num_before = mesh2->polygons.size();
    mesh2 = meshSubdivide(mesh2, thr);
    int num_after = mesh2->polygons.size();
    if (num_after == num_before) return mesh2;
  }
}

// Calculates the normals at each point on the surface.
void GeomUtils::calculateNormals(
    const pcl::PointCloud<Point_t>::Ptr xyz_cloud_in,
    pcl::PointCloud<pcl::PointNormal>::Ptr point_normal_cloud_out) {
  pcl::search::KdTree<Point_t>::Ptr tree(new pcl::search::KdTree<Point_t>);

  pcl::MovingLeastSquares<Point_t, pcl::PointNormal> moving_least_squares;

  moving_least_squares.setComputeNormals(set_compute_normals_);

  // Set parameters
  moving_least_squares.setInputCloud(xyz_cloud_in);
  moving_least_squares.setPolynomialFit(set_polynomial_fit_);
  moving_least_squares.setSearchMethod(tree);
  moving_least_squares.setSearchRadius(radius_search_);

  // Reconstruct
  moving_least_squares.process(*point_normal_cloud_out);
}

// Calculates the principle axis and the bounding box of an object.
void GeomUtils::calculatePCA(const pcl::PointCloud<Point_t>::Ptr cloud_in,
                             Eigen::Vector3f& position_out,
                             Eigen::Quaternionf& orientation_out,
                             Eigen::Vector3f& scale_out,
                             Eigen::Vector3f& mass_center_out) {
  pcl::MomentOfInertiaEstimation<Point_t> feature_extractor;
  feature_extractor.setInputCloud(cloud_in);
  feature_extractor.compute();

  Point_t min_point_AABB, max_point_AABB, min_point_OBB, max_point_OBB,
      position_OBB;

  Eigen::Matrix3f rotational_matrix_OBB;

  feature_extractor.getAABB(min_point_AABB, max_point_AABB);
  feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB,
                           rotational_matrix_OBB);

  feature_extractor.getMassCenter(mass_center_out);

  orientation_out = rotational_matrix_OBB;

  position_out[0] = position_OBB.x;
  position_out[1] = position_OBB.y;
  position_out[2] = position_OBB.z;

  scale_out[0] = max_point_OBB.x - min_point_OBB.x;
  scale_out[1] = max_point_OBB.y - min_point_OBB.y;
  scale_out[2] = max_point_OBB.z - min_point_OBB.z;
}

void GeomUtils::convertPolygonMeshToRosMesh(
    const pcl::PolygonMesh::Ptr& polygon_mesh_ptr,
    shape_msgs::Mesh::Ptr ros_mesh_ptr) {
  LOG(INFO) << "Conversion from PCL PolygonMesh to ROS Mesh started.";
  CHECK_NOTNULL(ros_mesh_ptr.get());
  CHECK_NOTNULL(polygon_mesh_ptr.get());
  CHECK_GT(polygon_mesh_ptr->polygons.size(), 0);
  CHECK_GT(polygon_mesh_ptr->cloud.width * polygon_mesh_ptr->cloud.height, 3);

  pcl_msgs::PolygonMesh pcl_msg_mesh;

  pcl_conversions::fromPCL(*polygon_mesh_ptr, pcl_msg_mesh);

  sensor_msgs::PointCloud2Modifier pcd_modifier(pcl_msg_mesh.cloud);

  size_t nbr_of_vertices = pcd_modifier.size();

  ros_mesh_ptr->vertices.resize(nbr_of_vertices);

  LOG(INFO) << "polys: " << pcl_msg_mesh.polygons.size()
            << " vertices: " << pcd_modifier.size();

  sensor_msgs::PointCloud2ConstIterator<float> pt_iter(pcl_msg_mesh.cloud, "x");

  for (size_t i = 0u; i < nbr_of_vertices; ++i, ++pt_iter) {
    ros_mesh_ptr->vertices[i].x = pt_iter[0];
    ros_mesh_ptr->vertices[i].y = pt_iter[1];
    ros_mesh_ptr->vertices[i].z = pt_iter[2];
  }

  LOG(INFO) << "Updated vertices";

  ros_mesh_ptr->triangles.resize(polygon_mesh_ptr->polygons.size());

  for (size_t i = 0u; i < polygon_mesh_ptr->polygons.size(); ++i) {
    if (polygon_mesh_ptr->polygons[i].vertices.size() < 3u) {
      LOG(WARNING) << "Not enough points in polygon. Ignoring it.";
      continue;
    }

    for (size_t j = 0u; j < 3u; ++j) {
      ros_mesh_ptr->triangles[i].vertex_indices[j] =
          polygon_mesh_ptr->polygons[i].vertices[j];
    }
  }
  LOG(INFO) << "Conversion from PCL PolygonMesh to ROS Mesh ended.";
}

}  // namespace object_discovery
