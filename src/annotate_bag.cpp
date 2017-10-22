#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <string>
#include <vector>

#include <pcl/io/vtk_lib_io.h>

#include "object_discovery/eigen_extensions.h"
#include "object_discovery/geom_utils.h"
#include "object_discovery/utils.h"
#include "object_discovery/vis_wrapper.h"

// Loads the entire bag, a result of running segment_scene.cpp on many scenes
// and allows for annotating segments as true or false
// Outputs a big NxD matrix fof measures for entire bag (as columns),
// together with an Nx1 vector of labels (+1 or -1).

// REQUIRES: segment_scene.cpp was run on some scenes (i.e. /measures and
// /segments is non-empty)
// REQUIRES: cooccurrence.cpp was run to fill in co-occurrence measure column

// PRODUCES: the labels "segmentLabels.eig.txt" vector via user interaction
int main(int argc, char** argv) {
  srand(1);

  object_discovery::Utils lib_utils;
  object_discovery::GeomUtils geom_utils;

  // highest id of any scene
  static const size_t kMaxSceneId = 70u;

  int loadLabels = 0u;
  if (argc > 1) loadLabels = atoi(argv[1]);

  // Go over all scenes, load all segments and measures
  std::vector<Cloud_t::Ptr> all_clouds;
  std::vector<NormalCloud_t::Ptr> all_normals;
  std::vector<size_t> scene_ids;
  size_t number_of_segments = 0u;
  size_t number_of_scenes = 0u;
  for (size_t i = 0; i < kMaxSceneId; ++i) {
    std::string plyfile = lib_utils.getScenePath(i);
    if (!lib_utils.file_exists(plyfile)) {
      // skip non-existing scene
      continue;
    }
    size_t segment_id = 0u;

    // Load in all segments as .ply files
    std::string segfile = lib_utils.getSegmentPath(i, segment_id);
    while (lib_utils.file_exists(segfile)) {
      Mesh_t::Ptr mesh(new Mesh_t);
      Cloud_t::Ptr cloud(new Cloud_t);
      NormalCloud_t::Ptr normals(new NormalCloud_t);
      pcl::io::loadPolygonFile(segfile, *mesh);
      geom_utils.convertMeshToPointsNormals(mesh, cloud, normals);
      all_clouds.push_back(cloud);
      all_normals.push_back(normals);
      scene_ids.push_back(i);

      ++segment_id;
      segfile = lib_utils.getSegmentPath(i, segment_id);
    }

    printf("Loaded scene %d/%d with %d segments...\n", i, kMaxSceneId,
           segment_id);
    number_of_segments += segment_id;
    number_of_scenes++;
  }
  printf("Number of segments: %d from %d scenes.\n", number_of_segments,
         number_of_scenes);

  // load measures from file (created by cooccurrence)
  std::string measuresPath =
      lib_utils.ROOT_PATH + std::string("segmentMeasures.eig.txt");
  Eigen::MatrixXf measures;
  eigen_extensions::loadASCII(measuresPath, &measures);
  assert(measures.rows() == number_of_segments);
  Eigen::VectorXf objectness =
      measures.rowwise().sum() /
      measures.cols();  // objectness is just the average

  // sort according to objectness (average across measures)
  std::vector<fipair> objv(number_of_segments);
  for (size_t i = 0u; i < number_of_segments; ++i) {
    objv[i] = fipair(objectness(i), i);
  }
  std::sort(objv.begin(), objv.end(),
            object_discovery::Utils::fiComparatorDescend);
  std::vector<Cloud_t::Ptr> cs;
  std::vector<NormalCloud_t::Ptr> ns;
  for (size_t i = 0u; i < number_of_segments; ++i) {
    int ix = objv[i].second;
    cs.push_back(all_clouds[ix]);
    ns.push_back(all_normals[ix]);
  }

  // Start the visualizer.
  VisWrapper visualizer;
  visualizer.vis_.setBackgroundColor(0.5, 0.5, 0.5);
  visualizer.vis_.setCameraPosition(0, 0, 0, 0.3, 0.3, 0.3, 0, -1, 0);

  // Show the cloud.
  geom_utils.showCloudsGridOriented(visualizer, cs, ns);

  // Initialize labels.
  // These are the paths for output (and input, in case of labels, potentially).
  std::string labelsPath =
      lib_utils.ROOT_PATH + std::string("segmentLabels.eig.txt");
  Eigen::VectorXf labels(number_of_segments);
  labels.setConstant(-1);
  if (loadLabels == 1) {
    if (lib_utils.file_exists(labelsPath)) {
      eigen_extensions::loadASCII(labelsPath, &labels);
      printf("loaded labels from file.\n");
    } else {
      printf(
          "attempt was made to load labels but the labels file doesn't "
          "exist.\n");
    }
  }

  // Show the labeling interface.
  int s = 0;
  Eigen::Quaternionf rot;
  rot.setIdentity();
  bool redraw = true;
  float space = 0.5;
  int gsize = static_cast<int>(sqrt(number_of_segments));
  while (true) {
    // draw selection boxes
    if (redraw) {
      visualizer.vis_.removeAllShapes();
      for (size_t i = 0u; i < number_of_segments; ++i) {
        int j = objv[i].second;
        if (labels(j) == 1) {
          Eigen::Vector3f tvec;
          tvec << (i / gsize) * space, (i % gsize) * space, 0;
          visualizer.vis_.addCube(tvec, rot, space, space, space,
                                    "box_" + std::to_string(i));
        }
      }
      redraw = false;
    }

    // Draw the selection box.
    visualizer.vis_.removeShape("selectionbox");
    Eigen::Vector3f tvec;
    tvec << (s / gsize) * space, (s % gsize) * space, 0;
    visualizer.vis_.addCube(tvec, rot, space / 2, space / 2, space / 2,
                              "selectionbox");

    // Print some info about selected segment.
    int ir = objv[s].second;
    cout << "Segment from scene " << scene_ids[ir]
         << ", measures: " << measures.row(ir) << endl;

    // Handle user intereraction.
    char c = visualizer.waitKey();
    int olds = s;
    if (c == 'q') break;
    if (c == 'w') {
      s -= 1;
    }
    if (c == 's') {
      s += 1;
    }
    if (c == 'a') {
      s -= gsize;
    }
    if (c == 'd') {
      s += gsize;
    }
    if (s < 0 || s >= number_of_segments) {
      s = olds;  // Reset it, invalid movement along grid.
    }

    if (c == 'x') {  // toggle label on this segment.
      if (labels(ir) == 1) {
        labels(ir) = -1;
        // We are toggling the box off, remove it.
        visualizer.vis_.removeShape("box_" + std::to_string(s));
      } else {
        labels(ir) = 1;
        Eigen::Vector3f tvec;
        tvec << (s / gsize) * space, (s % gsize) * space, 0;
        visualizer.vis_.addCube(tvec, rot, space, space, space,
                                  "box_" + std::to_string(s));
      }
    }
  }

  // save the labels and the normalized measures
  eigen_extensions::saveASCII(labels, labelsPath);
  printf("Saved labels and measures!!\n");
  printf("done.\n");
  return (0);
}
