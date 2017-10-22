/*
 * Segmenter.cpp
 *
 *  Created on: Dec 7, 2016
 *      Author: gocarlos
 */

#include "object_discovery/segmenter.hpp"

namespace object_discovery {

/*!
 * The values below are the default values.
 * ROS overwrite those parameters with values from
 * the ROS parameter server. When using the legacy executables, one can also
 * change those values there in their source code.
 */
Segmenter::Segmenter()
    : display_(true),
      keep_only_objects_above_main_plane_(true),
      plane_segmentation_distance_threshold_(0.02),
      vertices_cleaning_tolerance_(0.001f),
      main_plane_offset_distance_(0.01),
      order_of_objectness_(0u),
      max_iterations_plane_extraction_(5000u),
      min_number_of_inliers_(30u),
      dist_CoM_threshold_(0.08f),
      redraw_objectness_grid_(true),
      keep_upper_part_of_mesh_(false) {
  // segmentation parameters
  params.KTHRS.push_back(0.7);
  params.KTHRS.push_back(0.8);
  params.KTHRS.push_back(0.9);
  params.KTHRS.push_back(1.0);
  params.KTHRS.push_back(1.1);
  params.KTHRS.push_back(1.2);
  params.KTHRS.push_back(1.5);
  params.KTHRS.push_back(1.7);
  params.KTHRS.push_back(2.0);
  params.KTHRS.push_back(3.0);
  params.KTHRS.push_back(5.0);

  plane_point_cloud_.reset(new pcl::PointCloud<Point_t>);
}

bool Segmenter::segmentMesh(Mesh_t::Ptr& scene_mesh) {
  std::cout << "Segmentation process has begun!" << std::endl;

  // Clean all vectors first since this method could be executed multiple times.
  all_segments_.clear();

  // If mesh has some broken vertices or duplicate points, clean it.
  cleanPolygonMesh(scene_mesh);

  if (scene_mesh->cloud.width < 1u || scene_mesh->polygons.size() < 1u) {
    std::cerr << "Cloud or polygon vector seems to be empty?!" << std::endl;
    return false;
  }

  // Save the mesh as it is before any manipulation. This is done to show the
  // result after removing the main plane from the scene.
  Mesh_t::Ptr sceneMesh_unfiltered(new Mesh_t);
  *sceneMesh_unfiltered = *scene_mesh;

  Cloud_t::Ptr scene_cloud(new Cloud_t);
  NormalCloud_t::Ptr scene_normals(new NormalCloud_t);

  if (display_ == true) {
    pcl_visualizer_.reset(new VisWrapper);
    pcl_visualizer_->vis_.setBackgroundColor(0.5, 0.5, 0.5);
    pcl_visualizer_->vis_.setCameraPosition(0, 0, 0, 0.3, 0.3, 0.3, 0, -1, 0);
  }

  if (display_ == true) {
    printf(
        "Displaying the original input mesh. Press 'q' to "
        "continue\n");
    pcl_visualizer_->showMesh(scene_mesh);
    while (pcl_visualizer_->waitKey() != 'q') {
      continue;
    }
  }

  if (keep_only_objects_above_main_plane_) {
    // Remove everything below the supporting surface plane.
    // The supporting surface plane is the main plane found in the mesh e.g.
    // table plane.
    if (!removeSupportingSurface(scene_mesh)) {
      std::cout << "Could not remove the main plane from the scene."
                << std::endl;
    }
    if (display_ == true) {
      printf(
          "Going to segment supporting surface. Use 'w' to switch between the "
          "original and "
          "the mesh without supporting surface."
          "\n");
      pcl_visualizer_->showMesh(sceneMesh_unfiltered);
      bool escape = false;
      size_t scene_id = 0u;
      // This while loop is used to show the difference between the mesh after
      // removing everything below the main plane and the original.
      while (true) {
        char ch = pcl_visualizer_->waitKey();
        if (ch == 'q') {
          escape = true;
          break;
        }
        // When pressing 'w' the user can switch back and forth showing the
        // original and the mesh without main plane + everything below it.
        if (ch == 'w') {
          if (scene_id % 2 == 0) {
            printf(
                "Displaying the **unfiltered** scene Mesh. Press 'q' to "
                "continue or 'w' to compare\n");
            pcl_visualizer_->showMesh(sceneMesh_unfiltered);
          } else {
            printf(
                "Displaying the **filtered** scene Mesh. Press 'q' to "
                "continue "
                "or 'w' to compare\n");
            pcl_visualizer_->showMesh(scene_mesh);
          }
          ++scene_id;
        }
      }
    }
  }

  MAKECLOCK;
  TIC;
  // Get point cloud and point normal of the mesh. The point normals are
  // smoothified.
  geom_utils_.convertMeshToPointsNormals(scene_mesh, scene_cloud,
                                         scene_normals);
  TOC("got normals");

  for (size_t i = 0u; i < scene_cloud->points.size(); ++i) {
    Normal_t& n = scene_normals->points[i];
    if (std::isnan(n.normal_x) || std::isnan(n.normal_y) ||
        std::isnan(n.normal_z)) {
      scene_normals->points[i].normal_x = utils_.randf(-1, 1);
      scene_normals->points[i].normal_y = utils_.randf(-1, 1);
      scene_normals->points[i].normal_z = utils_.randf(-1, 1);
    }
  }

  if (display_ == true) {
    printf("Displaying the scene cloud. Press 'q' to continue\n");
    pcl_visualizer_->showCloud(scene_cloud);
    while (pcl_visualizer_->waitKey() != 'q') continue;
  }

  // Construct the edge graph for segmentation
  printf("Constructing edge graph based on mesh connectivity...\n");
  TIC;
  size_t number_of_polygons = scene_mesh->polygons.size();
  size_t number_of_points = scene_cloud->points.size();
  std::vector<std::pair<int, int>> es;
  geom_utils_.getEdges(scene_mesh, es);
  int number_of_edges = es.size();
  edge* edges = new edge[number_of_edges];
  for (size_t i = 0u; i < number_of_edges; ++i) {
    int a = es[i].first;
    int b = es[i].second;

    edges[i].a = a;
    edges[i].b = b;

    Normal_t& n1 = scene_normals->points[a];
    Normal_t& n2 = scene_normals->points[b];
    Point_t& p1 = scene_cloud->points[a];
    Point_t& p2 = scene_cloud->points[b];

    float dx = p2.x - p1.x;
    float dy = p2.y - p1.y;
    float dz = p2.z - p1.z;
    float dd = sqrt(dx * dx + dy * dy + dz * dz);
    dx /= dd;
    dy /= dd;
    dz /= dd;
    float dot = n1.normal_x * n2.normal_x + n1.normal_y * n2.normal_y +
                n1.normal_z * n2.normal_z;
    float dot2 = n2.normal_x * dx + n2.normal_y * dy + n2.normal_z * dz;
    float ww = 1.0f - dot;
    if (dot2 > 0.0f) {
      // Make it much less of a problem if convex regions have normal
      // difference.
      ww = ww * ww;
    }
    edges[i].w = ww;
  }
  TOC("made graph");

  // Go over all thresholds, segment, accumulate all segments into a bag
  printf("Segmenting scene with over many granularity thresholds...\n");
  // The associated normals.
  std::vector<NormalCloud_t::Ptr> all_segment_normals;
  // The point indeces. Will be used to compute union over intersection overlap.
  std::vector<std::set<int>> all_segment_pts;
  for (size_t kk = 0u; kk < params.KTHRS.size(); ++kk) {
    // Segment!
    float kthr = params.KTHRS[kk];
    TIC;
    universe* u = segment_graph(number_of_points, number_of_edges, kthr, edges);
    TOC("segmented");

    // Post-process segmentation by joining small segments.
    for (size_t j = 0u; j < number_of_edges; ++j) {
      int a = u->find(edges[j].a);
      int b = u->find(edges[j].b);
      if ((a != b) && ((u->size(a) < params.MINSEGSIZE) ||
                       (u->size(b) < params.MINSEGSIZE))) {
        u->join(a, b);
      }
    }

    // Display colored point cloud.
    if (display_ == true) {
      std::map<int, GeomUtils::RGBStruct> colmap;
      CloudColor_t::Ptr cres(new CloudColor_t);
      PointColor_t pt;
      GeomUtils::RGBStruct color;
      for (size_t q = 0u; q < number_of_points; ++q) {
        int comp = u->find(q);
        if (colmap.count(comp) > 0) {
          color = colmap[comp];
        } else {
          // add new color
          color.s.r = (rand() % 205) + 50;
          color.s.g = (rand() % 205) + 50;
          color.s.b = (rand() % 205) + 50;
          colmap[comp] = color;
        }

        pt.x = scene_cloud->points[q].x;
        pt.y = scene_cloud->points[q].y;
        pt.z = scene_cloud->points[q].z;
        pt.r = color.s.r;
        pt.g = color.s.g;
        pt.b = color.s.b;

        cres->push_back(pt);
      }
      printf("Going to display cres.\n");
      pcl_visualizer_->showCloud(cres);
      printf(
          "Displaying segmentation at threshold k=%f. Press 'q' to continue.\n",
          kthr);
      while (pcl_visualizer_->waitKey() != 'q') {
        continue;
      }
    }

    // Place all segments into their own cloud.
    TIC;
    std::vector<Cloud_t::Ptr> segments;
    std::vector<NormalCloud_t::Ptr> segment_normals;
    // A set of points are assigned to each segment, this is used for duplicate
    // detection.
    std::vector<std::set<int>> segment_pts;
    std::map<int, int> compToIx;
    for (size_t q = 0u; q < number_of_points; ++q) {
      int comp = u->find(q);
      int sz = u->size(comp);

      // Here hard thresholds are used on the number of points.
      // This is used for efficiency.
      if (sz < params.MINPTS) {
        continue;
      }  // This segment is small.
      if (sz > params.MAXPTS) {
        continue;
      }  // This segment is big.

      if (compToIx.count(comp) > 0) {
        // Add it on top of the (already created) cloud for this segment.
        int mapix = compToIx[comp];
        Cloud_t::Ptr& m = segments[mapix];
        m->push_back(scene_cloud->points[q]);
        ++m->height;

        NormalCloud_t::Ptr& nm = segment_normals[mapix];
        nm->push_back(scene_normals->points[q]);
        ++nm->height;

        segment_pts[compToIx[comp]].insert(q);

      } else {
        // Create new cloud for this segment.
        Cloud_t::Ptr m = Cloud_t::Ptr(new Cloud_t);
        m->height = 1;
        m->width = 1;
        m->push_back(scene_cloud->points[q]);
        segments.push_back(m);

        // Create new normals cloud for this segment.
        NormalCloud_t::Ptr nm = NormalCloud_t::Ptr(new NormalCloud_t);
        nm->height = 1;
        nm->width = 1;
        nm->push_back(scene_normals->points[q]);
        segment_normals.push_back(nm);

        // Create new set of indeces for this segment.
        std::set<int> s;
        s.insert(q);
        segment_pts.push_back(s);
        compToIx[comp] = segments.size() - 1;
      }
    }

    printf("Found %zu components using k=%f!\n", segments.size(), kthr);

    // Append into our bags that keep track of segments across all thresholds.
    for (size_t q = 0u; q < segments.size(); ++q) {
      all_segments_.push_back(segments[q]);
    }
    for (size_t q = 0u; q < segment_normals.size(); ++q) {
      all_segment_normals.push_back(segment_normals[q]);
    }
    for (size_t q = 0u; q < segment_pts.size(); ++q) {
      all_segment_pts.push_back(segment_pts[q]);
    }
  }  // End loop over segmentation thresholds.

  TOC("made segments");
  // Clean up this space, not needed anymore.
  delete[] edges;
  all_segments_before_projection_ = all_segments_;

  // Project all clouds to their eigenbasis.
  printf("Projecting all segments to their eigenbasis...\n");
  size_t number_of_all_segments = all_segments_.size();
  // Keep is an indicator vector of what segments we will keep at very end.
  std::vector<size_t> keep(number_of_all_segments, 1);
  // For visualization only, keeps track of segments we remove.
  std::vector<Cloud_t::Ptr> cloud_buffer;
  for (size_t i = 0u; i < number_of_all_segments; ++i) {
    float lambda1n, lambda2n;
    geom_utils_.projectToEigenbasis(all_segments_[i], all_segment_normals[i],
                                    all_segments_[i], all_segment_normals[i],
                                    lambda1n, lambda2n);

    // While we're at it, lets eliminate segments that are too thin or too flat.
    // (relatively or absolutely)
    if (lambda1n < params.THINTHR) {
      // This object is too thin.
      keep[i] = 0u;
      printf(
          "Eliminating segment %zu because eigvals(1)/eigvals(0)=%f, so it "
          "is too thin.\n",
          i, lambda1n);
      cloud_buffer.push_back(all_segments_[i]);
      continue;
    }
    if (lambda2n < params.FLATTHR) {
      // This object is too flat.
      keep[i] = 0u;
      printf(
          "Eliminating segment %zu because eigvals(2)/eigvals(0)=%f, so it "
          "is too flat.\n",
          i, lambda2n);
      cloud_buffer.push_back(all_segments_[i]);
      continue;
    }

    // The cloud is now in eigenbasis so ranges(0) is longest length and
    // ranges(2) is shortest length.
    Eigen::MatrixXf ptsmap = all_segments_[i]->getMatrixXfMap();
    Eigen::VectorXf mins = ptsmap.rowwise().minCoeff();
    Eigen::VectorXf maxes = ptsmap.rowwise().maxCoeff();
    Eigen::VectorXf ranges = maxes - mins;
    float vx = ranges(0);
    float vy = ranges(1);
    float vz = ranges(2);
    if (vx < params.MINLENGTH || vy < params.MINLENGTH ||
        vz < params.MINLENGTH || vx > params.MAXLENGTH ||
        vy > params.MAXLENGTH || vz > params.MAXLENGTH) {
      keep[i] = 0u;
      printf(
          "Eliminating cloud %zu because it's too small/big in some direction "
          "%f %f %f\n",
          i, vx, vy, vz);
      cloud_buffer.push_back(all_segments_[i]);
      continue;
    }
  }

  if (display_ == true) {
    printf(
        "Displaying segments that were rejected due to dimensions. Press "
        "'q' to continue.\n");
    geom_utils_.showCloudsGrid(*pcl_visualizer_, cloud_buffer);
    while (pcl_visualizer_->waitKey() != 'q') {
      continue;
    }
  }

  // Compute objectness measures on all segments.
  printf("Computing shape objectness measures on all segments...\n");
  constexpr int num_measures = 5;
  Eigen::MatrixXf measures(number_of_all_segments, num_measures);
  measures.setZero();
  Eigen::MatrixXf mrows(number_of_all_segments, num_measures);
  mrows.setZero();
  Eigen::VectorXf times(num_measures);
  times.setZero();
  size_t np = 0u;
  for (size_t i = 0u; i < number_of_all_segments; ++i) {
    if (i % 5 == 0) {
      printf("Processing segment %zu/%zu\n", i, number_of_all_segments);
    }
    if (keep[i] == 0u) {
      continue;
    }
    double s;
    s = pcl::getTime();
    measures(i, 0) = measures_.scoreCompactness(all_segments_[i]);
    times(0) += pcl::getTime() - s;
    s = pcl::getTime();
    measures(i, 1) =
        measures_.scoreSymmetry(all_segments_[i], all_segment_normals[i],
                                params.SYMMETRY_MEASURE_CLOUD_NORMALS_TRADEOFF);
    times(1) += pcl::getTime() - s;
    s = pcl::getTime();
    measures(i, 2) = measures_.scoreGlobalConvexity(all_segments_[i]);
    times(2) += pcl::getTime() - s;
    s = pcl::getTime();
    measures(i, 3) =
        measures_.scoreLocalConvexity(all_segments_[i], all_segment_normals[i],
                                      params.LOCAL_CONVX_MEASURE_NNRADIUS);
    times(3) += pcl::getTime() - s;
    s = pcl::getTime();
    measures(i, 4) = measures_.scoreSmoothness(
        all_segments_[i], all_segment_normals[i],
        params.SMOOTHNESS_MEASURE_NNRADIUS, params.SMOOTHNESS_MEASURE_NUMBINS);
    times(4) += pcl::getTime() - s;
    mrows.row(np) = measures.row(i);
    ++np;
  }
  times /= 0.001f * np;
  printf("Average compute times (in milliseconds) per segment:\n");
  printf("compactness: %.1fms\n", times(0));
  printf("symmetry: %.1fms\n", times(1));
  printf("global convexity: %.1fms\n", times(2));
  printf("local convexity: %.1fms\n", times(3));
  printf("smoothness: %.1fms\n", times(4));

  // Compute mean and standard deviation along each column.
  Eigen::MatrixXf mm = mrows.block(0, 0, np, num_measures);
  Eigen::VectorXf mmean = mm.colwise().sum() / mm.rows();
  mm.rowwise() -= mmean.transpose();
  Eigen::MatrixXf mstd = (mm.colwise().squaredNorm() / mm.rows()).cwiseSqrt();

  // Normalize the objectness scores.
  measures_normalized_ = measures;
  measures_normalized_.rowwise() -= mmean.transpose();
  measures_normalized_ = measures_normalized_.cwiseQuotient(
      mstd.replicate(measures_normalized_.rows(), 1));
  // Objectness is just the average.
  Eigen::VectorXf objectness =
      measures_normalized_.rowwise().sum() / measures_normalized_.cols();

  // Carry out non-maximum suppression.
  printf(
      "Eliminating duplicate segments using non-max suppression based on "
      "objectness...\n");
  TIC;
  size_t ntot = 0u;
  size_t nevaled = 0u;
  for (size_t i = 0u; i < all_segment_pts.size(); ++i) {
    std::set<int>& s1 = all_segment_pts[i];
    for (size_t j = 0u; j < all_segment_pts.size(); ++j) {
      if (i == j) {
        continue;
      }
      if (keep[i] == 0u) {
        continue;
      }
      if (keep[j] == 0u) {
        continue;
      }
      std::set<int>& s2 = all_segment_pts[j];

      float s1s = s1.size();
      float s2s = s2.size();
      float bestCaseIOU = std::min(s1s, s2s) / std::max(s1s, s2s);
      ++ntot;

      if (bestCaseIOU > params.IOUTHR) {
        ++nevaled;
        // Do a double check.
        std::set<int> intersect, setunion;
        set_intersection(s1.begin(), s1.end(), s2.begin(), s2.end(),
                         inserter(intersect, intersect.begin()));
        set_union(s1.begin(), s1.end(), s2.begin(), s2.end(),
                  inserter(setunion, setunion.begin()));

        float IOU = (1.0f * intersect.size()) / setunion.size();
        if (IOU > params.IOUTHR) {
          // Too much overlap, retain the one with higher objectness
          // also give advantage to bigger segments.
          if (objectness(i) > objectness(j)) {
            keep[j] = 0u;
            printf("%zu is suppressing %zu because %f > %f\n", i, j,
                   objectness(i), objectness(j));
          } else {
            keep[i] = 0u;
            printf("%zu is suppressing %zu because %f <= %f\n", j, i,
                   objectness(i), objectness(j));
          }
        }
      }
    }
  }
  printf(
      "Total number of duplicate checks: %zu, but only %zu were expensively "
      "considered.\n",
      ntot, nevaled);
  TOC("non-max suppression computed");

  // Show final objects found in this scene highlighted in the scene.
  if (display_ == true) {
    printf(
        "Displaying top segments found in the scene. Use 'w'/'s' to show "
        "more or less, and use 'q' to continue.\n");

    // Sort all segments by objectness.
    std::vector<fipair> objv(number_of_all_segments);
    for (size_t i = 0u; i < number_of_all_segments; ++i) {
      objv[i] = fipair(objectness(i), i);
    }
    std::sort(objv.begin(), objv.end(), Utils::fiComparatorDescend);

    int num_want = 0;
    Cloud_t::Ptr sc(new Cloud_t(*scene_cloud));
    pcl_visualizer_->holdOff();
    while (true) {
      // Dim the original scene cloud.
      constexpr float dimratio = 0.2f;
#ifdef USE_COLOR
      for (size_t i = 0u; i < sc->points.size(); ++i) {
        sc->points[i].r = floor(dimratio * scene_cloud->points[i].r);
        sc->points[i].g = floor(dimratio * scene_cloud->points[i].g);
        sc->points[i].b = floor(dimratio * scene_cloud->points[i].b);
      }
#endif
      // Iterate over segments and make their points visible in original.
      size_t num_showed = 0u;
      for (size_t iii = 0u; iii < all_segment_pts.size(); ++iii) {
        size_t ii = objv[iii].second;
        if (keep[ii] == 0u) continue;

        for (std::set<int>::const_iterator it = all_segment_pts[ii].begin();
             it != all_segment_pts[ii].end(); it++) {
          int i = *it;
#ifdef USE_COLOR
          // If all there are no colors, use white.
          if (scene_cloud->points[i].r == 0 && scene_cloud->points[i].g == 0 &&
              scene_cloud->points[i].b == 0) {
            sc->points[i].r = 255;
            sc->points[i].g = 255;
            sc->points[i].b = 255;
          } else {
            sc->points[i].r = scene_cloud->points[i].r;
            sc->points[i].g = scene_cloud->points[i].g;
            sc->points[i].b = scene_cloud->points[i].b;
          }
#endif
        }
        ++num_showed;
        if (num_showed >= num_want) break;
      }

      pcl_visualizer_->showCloud(sc);
      bool escape = false;
      while (true) {
        char ch = pcl_visualizer_->waitKey();
        if (ch == 'q') {
          escape = true;
          break;
        }
        if (ch == 'w') {
          ++num_want;
          break;
        }
        if (ch == 's') {
          --num_want;
          break;
        }
      }
      if (num_want < 0) {
        num_want = 0;
      }
      if (escape) {
        break;
      }
    }
  }

  if (display_ == true) {
    printf("Displaying grid of segments in order of decreasing objectness.\n");

    while (true) {
      if (redraw_objectness_grid_) {
        // Sort by objectness.
        std::vector<fipair> objv(number_of_all_segments);
        printf(
            "0: Average, 1: Compactness, 2: Symmetry, 3: Global Convexity, "
            "4: Local Convexity, 5: Smoothness\n");
        printf("Currently sorted by %zu\n", order_of_objectness_ % 6);
        if (order_of_objectness_ % 6 == 0) {
          for (size_t i = 0u; i < number_of_all_segments; ++i) {
            objv[i] = fipair(objectness(i), i);
          }
        }
        if (order_of_objectness_ % 6 == 1) {
          for (size_t i = 0u; i < number_of_all_segments; ++i) {
            objv[i] = fipair(measures_normalized_(i, 0), i);
          }
        }
        if (order_of_objectness_ % 6 == 2) {
          for (size_t i = 0u; i < number_of_all_segments; ++i) {
            objv[i] = fipair(measures_normalized_(i, 1), i);
          }
        }
        if (order_of_objectness_ % 6 == 3) {
          for (size_t i = 0u; i < number_of_all_segments; ++i) {
            objv[i] = fipair(measures_normalized_(i, 2), i);
          }
        }
        if (order_of_objectness_ % 6 == 4) {
          for (size_t i = 0u; i < number_of_all_segments; ++i) {
            objv[i] = fipair(measures_normalized_(i, 3), i);
          }
        }
        if (order_of_objectness_ % 6 == 5) {
          for (size_t i = 0u; i < number_of_all_segments; ++i) {
            objv[i] = fipair(measures_normalized_(i, 4), i);
          }
        }
        std::sort(objv.begin(), objv.end(), Utils::fiComparatorDescend);

        std::vector<Cloud_t::Ptr> clouds_to_show;
        std::vector<NormalCloud_t::Ptr> normals_to_show;
        for (size_t ii = 0u; ii < number_of_all_segments; ++ii) {
          int i = objv[ii].second;
          if (keep[i] == 0u) {
            continue;
          }
          clouds_to_show.push_back(all_segments_[i]);
          normals_to_show.push_back(all_segment_normals[i]);
          printf("Showing cloud %zu with value %f\n", ii, objv[ii].first);
        }
        geom_utils_.showCloudsGridOriented(*pcl_visualizer_, clouds_to_show,
                                           normals_to_show);
      }
      redraw_objectness_grid_ = false;
      char ch = pcl_visualizer_->waitKey();
      if (ch == 'q') break;
      if (ch == 'w') {
        ++order_of_objectness_;
        redraw_objectness_grid_ = true;
      }  // Cycle through objectness measures.
      if (ch == 's') {
        --order_of_objectness_;
        redraw_objectness_grid_ = true;
      }
    }
  }

  // Now we have to do some annoying book-keeping. We have clouds, indeces, and
  // normals of all segments and keep[] indicator vector is telling us which
  // ones we want to keep. We need to use the original mesh to pluck out these
  // segments into their own little mesh segments so that we can save them.
  printf("Creating meshes for all segments...\n");

  // First create a lookup map of vertex -> indices of all polygons each vertex
  // is involved in.
  TIC;
  std::map<int, std::vector<int>> vmap;
  for (size_t j = 0u; j < scene_mesh->polygons.size(); ++j) {
    const pcl::Vertices& v = scene_mesh->polygons[j];
    int v1 = v.vertices[0];
    int v2 = v.vertices[1];
    int v3 = v.vertices[2];
    vmap[v1].push_back(j);
    vmap[v2].push_back(j);
    vmap[v3].push_back(j);
  }

  // Now stitch all mesh segments together.
  meshes_.resize(number_of_all_segments);
  for (size_t i = 0u; i < number_of_all_segments; ++i) {
    if (keep[i] == 0u) {
      continue;
    }

    // We are keeping this segment, so convert it to mesh.
    Mesh_t::Ptr new_mesh(new Mesh_t);
    std::set<int>& ss = all_segment_pts[i];
    Cloud_t::Ptr currpts = all_segments_[i];

    // Pass 1: For all points in this segment, copy over the vertices from
    // scene_cloud.
    std::map<int, int> ixmap;
    size_t nc = 0u;
    for (std::set<int>::const_iterator it = ss.begin(); ss.end() != it; ++it) {
      ixmap[*it] = nc;
      ++nc;
    }

    // Pass 2: Count up for polygons, we only want to copy over those st all
    // points are in our set.
    std::vector<int> counts(scene_mesh->polygons.size(), 0);
    for (std::set<int>::const_iterator it = ss.begin(); it != ss.end(); ++it) {
      // Get indeces of all polygons this vertex is in.
      std::vector<int>& vi = vmap[*it];
      for (size_t j = 0u; j < vi.size(); ++j) {
        ++counts[vi[j]];
      }
    }

    // Pass 3: Copy over the associated polygons, get indeces of all polygons
    // this vertex is in.
    for (std::set<int>::const_iterator it = ss.begin(); it != ss.end(); ++it) {
      std::vector<int>& vi = vmap[*it];
      for (size_t j = 0u; j < vi.size(); ++j) {
        if (counts[vi[j]] < 3) continue;

        pcl::Vertices& v = scene_mesh->polygons[vi[j]];
        pcl::Vertices pnew;
        pnew.vertices.resize(3);
        pnew.vertices[0] = ixmap[v.vertices[0]];
        pnew.vertices[1] = ixmap[v.vertices[1]];
        pnew.vertices[2] = ixmap[v.vertices[2]];

        new_mesh->polygons.push_back(pnew);
        // Set the polygon count to 0, such that each polygon gets added only
        // once.
        counts[vi[j]] = 0;
      }
    }
    pcl::toPCLPointCloud2(*(all_segments_before_projection_[i]),
                          new_mesh->cloud);

    meshes_[i] = new_mesh;
  }
  TOC("Stitching done.");

  kept_objects_ = keep;
  LOG(INFO) << "Kept objects: " << kept_objects_.size();

  LOG(INFO) << "Segmentation Process has ended!";

  return true;
}

// This method segments the main plane found on the mesh and removes the plane
// including everything under it. To calculate which part should be kept
// and which part should be removed, the center of mass of the mesh is
// calculated and the side of the plane which contains the center of mass is
// kept, the other side removed. To make the center of mass more robust against
// big outliers like parts of the floor or walls on the scene only points close
// to the main plane are taken into account when calculating the CoM. Normally
// nothing right below the table is captured and therefore only objects on the
// table and the table itself affect CoM calculation --> CoM is near the objects
// above the table.

// TODO(gocarlos): one could also set over a parameter that the objects are on a
// surface above the table and that the objects are above the surface in
// z-direction.
bool Segmenter::removeSupportingSurface(Mesh_t::Ptr& mesh_in) {
  // Get the plane coefficients.
  Cloud_t::Ptr cloud_in(new Cloud_t);
  pcl::fromPCLPointCloud2(mesh_in->cloud, *cloud_in);

  std::cout
      << "Number of polygons in mesh before removing the supporting surface "
         "and everything under it: "
      << mesh_in->polygons.size() << std::endl;

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  // Create the segmentation object
  pcl::SACSegmentation<Point_t> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMaxIterations(max_iterations_plane_extraction_);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(plane_segmentation_distance_threshold_);

  seg.setInputCloud(cloud_in);
  seg.segment(*inliers, *coefficients);

  // General Plane form: ax + by + cz + d = 0
  double a, b, c, d;
  a = coefficients->values[0];
  b = coefficients->values[1];
  c = coefficients->values[2];
  d = coefficients->values[3];

  if (inliers->indices.size() <= min_number_of_inliers_) {
    std::cerr << "Could not estimate a planar model for the given dataset.";
    return false;
  } else {
    std::cout << "Model coefficients:" << coefficients->values[0] << "*x + "
              << coefficients->values[1] << "*y + " << coefficients->values[2]
              << "*z + " << coefficients->values[3] << " = 0" << std::endl;

    std::cout << "Model inliers: " << inliers->indices.size() << std::endl;

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<Point_t> extract;
    extract.setInputCloud(cloud_in);
    extract.setIndices(inliers);
    extract.setNegative(false);

    // Get the points associated with the planar surface
    extract.filter(*plane_point_cloud_);
    std::cout << "PointCloud representing the planar component: "
              << plane_point_cloud_->points.size() << " data points."
              << std::endl;
  }

  // Position of a point on the plane.
  Eigen::Vector3d plane_position;
  constexpr double precision = 0.0000001;
  if (std::abs(c) >= precision) {
    plane_position[0] = 0.0f;
    plane_position[1] = 0.0f;
    plane_position[2] = -d / c;
  } else if (std::abs(a) >= precision) {
    plane_position[0] = -d / a;
    plane_position[1] = 0.0f;
    plane_position[2] = 0.0f;
  } else if (std::abs(b) >= precision) {
    plane_position[0] = 0.0f;
    plane_position[1] = -d / b;
    plane_position[2] = 0.0f;
  }

  // Normal to the plane to one side.
  Eigen::Vector3d plane_normal;
  double normal_norm;
  normal_norm = (sqrt(pow(a, 2.0) + pow(b, 2.0) + pow(c, 2.0)));
  plane_normal[0] = a / normal_norm;
  plane_normal[1] = b / normal_norm;
  plane_normal[2] = c / normal_norm;

  if (keep_upper_part_of_mesh_) {
    LOG(INFO)
        << "Slicing the mesh and keeping the part in positive z direction.";
    if (plane_normal[2] < 0) {
      plane_normal *= -1.0f;
    }
  } else {
    // In order to know which side of the main plane should be removed, one
    // calculates the center of mass from the point cloud.
    // In order to not let some unwanted points (floor, wall etc.) affect the
    // CoM,
    // all points further away then some threshold from the main plane are cut
    // away.
    Mesh_t::Ptr mesh_around_main_plane(new Mesh_t);
    sliceMesh(mesh_in, plane_position + plane_normal * dist_CoM_threshold_,
              -plane_normal, mesh_around_main_plane);
    sliceMesh(mesh_around_main_plane,
              plane_position - plane_normal * dist_CoM_threshold_, plane_normal,
              mesh_around_main_plane);

    if (display_ == true) {
      printf(
          "Displaying the scene mesh around the main plane. Press 'q' to "
          "continue\n");
      pcl_visualizer_->showMesh(mesh_around_main_plane);
      while (pcl_visualizer_->waitKey() != 'q') continue;
    }

    Cloud_t::Ptr cloud_around_main_plane(new Cloud_t);
    pcl::fromPCLPointCloud2(mesh_around_main_plane->cloud,
                            *cloud_around_main_plane);

    Eigen::Quaternionf cloud_orientation;
    Eigen::Vector3f cloud_scale, cloud_position, mass_center;

    geom_utils_.calculatePCA(cloud_around_main_plane, cloud_position,
                             cloud_orientation, cloud_scale, mass_center);

    // Generate two points on each side of the plane.
    // Point on plane + normal in one direction.
    Eigen::Vector3f point_on_first_direction;
    point_on_first_direction[0] = 0.0f + plane_normal[0];
    point_on_first_direction[1] = 0.0f + plane_normal[1];
    point_on_first_direction[2] = (-d / c) + plane_normal[2];
    // Point on plane - normal in one direction.
    Eigen::Vector3f point_on_second_direction;
    point_on_second_direction[0] = 0.0f - plane_normal[0];
    point_on_second_direction[1] = 0.0f - plane_normal[1];
    point_on_second_direction[2] = (-d / c) - plane_normal[2];

    // Get distance from those two points to the center of mass.
    float distance_to_first =
        (sqrt(pow(point_on_first_direction[0] - mass_center[0], 2.0) +
              pow(point_on_first_direction[1] - mass_center[1], 2.0) +
              pow(point_on_first_direction[2] - mass_center[2], 2.0)));
    float distance_to_second =
        (sqrt(pow(point_on_second_direction[0] - mass_center[0], 2.0) +
              pow(point_on_second_direction[1] - mass_center[1], 2.0) +
              pow(point_on_second_direction[2] - mass_center[2], 2.0)));

    // The point which is closer to the center of mass is on the side, which
    // should be kept.
    // Invert the normal direction if CoM is closer to the second point.
    if (distance_to_first > distance_to_second) {
      plane_normal *= -1.0f;
    }
  }

  // Set new plane position as old plane position + with an offset in the
  // direction to the normal = new plane is parallel to the old one.
  Eigen::Vector3d plane_offset;
  plane_offset[0] = plane_normal[0] * main_plane_offset_distance_;
  plane_offset[1] = plane_normal[1] * main_plane_offset_distance_;
  plane_offset[2] = plane_normal[2] * main_plane_offset_distance_;

  // Remove the main plane and everything below it from the final mesh.
  sliceMesh(mesh_in, plane_position + plane_offset, plane_normal, mesh_in);

  std::cout
      << "Number of polygons in mesh after removing the supporting surface "
         "and everything under it: "
      << mesh_in->polygons.size() << std::endl;
  return true;
}

void Segmenter::sliceMesh(const pcl::PolygonMesh::Ptr mesh_in,
                          const Eigen::Vector3d plane_position_in,
                          const Eigen::Vector3d plane_normal_in,
                          pcl::PolygonMesh::Ptr mesh_out) {
  // Convert the mesh in a vtk polydata type, cut it with the plane and remove
  // everything in the normal direction.
  vtkSmartPointer<vtkPolyData> poly_data = vtkSmartPointer<vtkPolyData>::New();
  vtkSmartPointer<vtkPolyData> poly_data_temp =
      vtkSmartPointer<vtkPolyData>::New();
  pcl::io::mesh2vtk(*mesh_in, poly_data_temp);

  // Define a clipping plane
  vtkSmartPointer<vtkPlane> clipPlane = vtkSmartPointer<vtkPlane>::New();
  clipPlane->SetOrigin(plane_position_in[0], plane_position_in[1],
                       plane_position_in[2]);
  clipPlane->SetNormal(plane_normal_in[0], plane_normal_in[1],
                       plane_normal_in[2]);

  // Clip the source with the plane
  vtkSmartPointer<vtkClipPolyData> clipper =
      vtkSmartPointer<vtkClipPolyData>::New();
#if VTK_MAJOR_VERSION <= 5
  clipper->SetInput(poly_data_temp);
#else
  clipper->SetInputData(poly_data_temp);
#endif
  clipper->SetClipFunction(clipPlane);
  clipper->Update();
  clipper->GenerateClippedOutputOn();
  poly_data = clipper->GetOutput();

  // Convert the polydata back to PCL polygonmesh.
  pcl::io::vtk2mesh(poly_data, *mesh_out);
}

void Segmenter::cleanPolygonMesh(pcl::PolygonMesh::Ptr polygon_mesh) {
  std::cout << "Going to clean the PolygonMesh.\n" << std::endl;
  std::cout << "Cleaning Tolerance: " << vertices_cleaning_tolerance_
            << std::endl;

  vtkSmartPointer<vtkPolyData> poly_data = vtkSmartPointer<vtkPolyData>::New();
  vtkSmartPointer<vtkPolyData> poly_data_temp =
      vtkSmartPointer<vtkPolyData>::New();
  pcl::io::mesh2vtk(*polygon_mesh, poly_data_temp);

  std::cout << "Input mesh has " << poly_data_temp->GetNumberOfPoints()
            << " vertices." << std::endl;

  vtkSmartPointer<vtkCleanPolyData> cleanPolyData =
      vtkSmartPointer<vtkCleanPolyData>::New();

#if VTK_MAJOR_VERSION <= 5
  cleanPolyData->SetInput(poly_data_temp);
#else
  cleanPolyData->SetInputData(poly_data_temp);
#endif

  cleanPolyData->SetTolerance(vertices_cleaning_tolerance_);
  cleanPolyData->Update();

  std::cout << "Cleaned mesh has "
            << cleanPolyData->GetOutput()->GetNumberOfPoints() << " vertices."
            << std::endl;

  poly_data->DeepCopy(cleanPolyData->GetOutput());

  pcl::io::vtk2mesh(poly_data, *polygon_mesh);
  std::cout << "Input mesh has " << std::endl;

  std::cout << "Cleaning of PolygonMesh done.";
}

}  // namespace object_discovery
