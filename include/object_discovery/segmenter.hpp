/*
 * Segmenter.hpp
 *
 *  Created on: Dec 7, 2016
 *      Author: gocarlos
 */

#pragma once

#include <glog/logging.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <map>
#include <set>
#include <utility>
#include <vector>

#include <pcl/filters/extract_indices.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <vtkCleanPolyData.h>
#include <vtkClipPolyData.h>
#include <vtkPlane.h>

#include "object_discovery/eigen_extensions.h"
#include "object_discovery/geom_utils.h"
#include "object_discovery/measures.h"
#include "object_discovery/segment_graph.h"  // felzenswalb segmenter
#include "object_discovery/utils.h"
#include "object_discovery/vis_wrapper.h"

namespace object_discovery {

class Segmenter {
 public:
  /*!
   * Constructor.
   */
  Segmenter();

  //! Segment mesh which is given as a pointer.
  bool segmentMesh(Mesh_t::Ptr& sceneMesh);

  /*!
   * Remove the supporting surface aka the main plane in the scene as well as
   * everything else below the supporting surface. More details in the
   * implementation
   */
  bool removeSupportingSurface(Mesh_t::Ptr& mesh_in);

  /*!
   * If the input mesh has duplicate points or the vertices indices are not
   * correct this method can correct those errors.
   */
  void cleanPolygonMesh(pcl::PolygonMesh::Ptr polygon_mesh);

  /*!
   * This method cuts the given PCL polygonmesh with a given plane, which
   * is build with a point in this plane and the normal to this plane. The kept
   * part of the plane is the one to which the normal points.
   * @param mesh_in
   * @param plane_position_in
   * @param plane_normal_in
   * @param mesh_out
   */
  void sliceMesh(const pcl::PolygonMesh::Ptr mesh_in,
                 const Eigen::Vector3d plane_position_in,
                 const Eigen::Vector3d plane_normal_in,
                 pcl::PolygonMesh::Ptr mesh_out);

  //! Will store all clouds across all thresholds
  std::vector<Cloud_t::Ptr> all_segments_;
  std::vector<Mesh_t::Ptr> meshes_;

  //! Will store all clouds across all thresholds before projection
  std::vector<Cloud_t::Ptr> all_segments_before_projection_;

  /*!
   * Some file system utilities from this class.
   */
  Utils utils_;

  /*!
   * Some cloud manipulation and visualization methods from this class.
   */
  GeomUtils geom_utils_;

  /*!
   * Class which provides methods to score the segments found in the scene.
   */
  Measures measures_;

  //! Display the whole operation with the PCL Viewer.
  bool display_;

  //! Redraw grid showing objects positioned after objectness.
  bool redraw_objectness_grid_;

  //! Pointer to the PCL Viewer.
  VisWrapper::Ptr pcl_visualizer_;

  /*!
   * Parameter for the ordering of the objects when they are displayid as a grid
   * 0: Average, 1: Compactness, 2: Symmetry, 3: Global Convexity, 4: Local
   * Convexity, 5: Smoothness
   */
  size_t order_of_objectness_;

  /*!
   * Matrix containing all the measures done for each object.
   * These are:
   * Average; Compactness; Symmetry; Global convexity; Local convexity;
   * Smoothness
   */
  Eigen::MatrixXf measures_normalized_;

  /*!
   * Is an indicator vector of what segments will keep at very end.
   */
  std::vector<size_t> kept_objects_;

  /*!
   * If true the main plane is removed from the input mesh when segmenting the
   * objects.
   */
  bool keep_only_objects_above_main_plane_;

  /*!
   * Distance in meters to the model of the plane when segmenting it.
   * see setDistanceThreshold in sac_segmentation.h from PCL.
   */
  double plane_segmentation_distance_threshold_;

  /*!
   * If this is set to true then the part of the segmented main plain pointing
   * in +z direction is kept.
   */
  bool keep_upper_part_of_mesh_;

  /*!
   * Tolerance for the distance between the vertices for cleaning duplicates.
   * not in millimeters but as a fraction of the bounding box of the mesh.
   */
  float vertices_cleaning_tolerance_;

  /*!
   * Offset in meters from the main plane to where the mesh is sliced.
   * Slice is then parallel to the main plane with the offset. By adding the
   * offset, all points from the main plane are removed.
   */
  double main_plane_offset_distance_;

  //! Point cloud which represents the main plane.
  pcl::PointCloud<Point_t>::Ptr plane_point_cloud_;

  size_t max_iterations_plane_extraction_;
  /*!
   * Minimal of inliers in order to remove the main plane. One has to have min 3
   * inliers to build a plane but a higher number of inliers means that the
   * plane found is better.
   */
  size_t min_number_of_inliers_;

  /*!
   * When calculating the center of mass of a point cloud, only take points
   * near then dist_CoM_threshold_ from a given plane into consideration.
   * See implementation of removeSupportingSurface for more information.
   */
  float dist_CoM_threshold_;

  //! Segmentation parameters
  struct SegmentationParams {
    SegmentationParams()
        : MINSEGSIZE(1000),
          MINPTS(500),
          MAXPTS(50000),
          THINTHR(0.01),
          FLATTHR(0.005),
          MINLENGTH(0.05),
          MAXLENGTH(0.80),
          SYMMETRY_MEASURE_CLOUD_NORMALS_TRADEOFF(0.3),
          LOCAL_CONVX_MEASURE_NNRADIUS(0.0075),
          SMOOTHNESS_MEASURE_NNRADIUS(0.01),
          SMOOTHNESS_MEASURE_NUMBINS(8),
          IOUTHR(0.8) {}

    std::vector<float> KTHRS;

    /*!
     * Minimum segment size, or it is merged to neighboring clusters during
     * segmentation
     */
    int MINSEGSIZE;

    // HARD OBJECTNESS PARAMETERS
    /*!
     * Max and min number of points. These two are mostly just
     * for efficiency
     */
    int MINPTS;
    int MAXPTS;

    float THINTHR;
    float FLATTHR;

    /*!
     * Minimum extent of a segment along minimal direction (in meters)
     */
    float MINLENGTH;

    /*!
     * Maximum extent of a segment along maximal direction (in meters)
     */
    float MAXLENGTH;

    // MEASURE PARAMETERS
    /*!
     * Scaling factor for difference in normals wrt. difference in position
     * for points, when computing difference between two point clouds.
     * 0 => Only look at difference between point and its closest point
     */
    float SYMMETRY_MEASURE_CLOUD_NORMALS_TRADEOFF;

    /*!
     * Higher value => matching normals are more important than point distances
     * used in Local Convexity measure for local neighborhood
     */
    float LOCAL_CONVX_MEASURE_NNRADIUS;

    /*!
     * Used in Smoothness measure for local neighborhood finding
     */
    float SMOOTHNESS_MEASURE_NNRADIUS;

    /*!
     * Number of bins in histogram. We found 8 to work best quite consistently.
     */
    float SMOOTHNESS_MEASURE_NUMBINS;

    /*!
     * Non-max suppression parameters intersection over union threshold, during
     * non-max suppression
     */
    float IOUTHR;
  } params;
};

}  // namespace object_discovery
