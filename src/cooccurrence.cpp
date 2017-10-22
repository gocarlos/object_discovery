#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <pcl/io/vtk_lib_io.h>

#include "object_discovery/eigen_extensions.h"
#include "object_discovery/geom_utils.h"
#include "object_discovery/utils.h"
#include "object_discovery/vis_wrapper.h"

// REQUIRES: segment_scene.cpp was run on some scenes (i.e. /measures and
// /segments is non-empty)
// PRODUCES: segmentMeasures.eig.txt (contains all measures, including
//           co-occurrence for all segments concatenated in a single big matrix)
int main(int argc, char** argv) {
  srand(1);

  // Using methods contained in those classes.
  object_discovery::Utils utils;
  object_discovery::GeomUtils geom_utils;

  // Highest id of any scene.
  size_t kMaxSceneID = 70u;
  float kMultply = 0.25f;
  // Number of nearest neighbors to consider when computing co-occurence score.
  size_t kNumberOfNeigbors = 10u;
  int display = 0;
  if (argc > 1) display = atoi(argv[1]);

  // Go over all scenes, load all segments and measures
  Eigen::MatrixXf measures;
  std::vector<Cloud_t::Ptr> all_clouds;
  std::vector<NormalCloud_t::Ptr> all_normals;
  std::vector<int> scene_ids;
  size_t number_of_segments = 0u;
  size_t number_of_scenes = 0u;
  for (size_t i = 0u; i < kMaxSceneID; ++i) {
    std::string plyfile = utils.getScenePath(i);
    if (!utils.file_exists(plyfile)) {
      // Skip non-existing scene.
      continue;
    }

    // Load in all segments as .ply files
    size_t segment_id = 0u;
    std::string segfile = utils.getSegmentPath(i, segment_id);
    while (utils.file_exists(segfile)) {
      Mesh_t::Ptr mesh(new Mesh_t);
      Cloud_t::Ptr cloud(new Cloud_t);
      NormalCloud_t::Ptr normals(new NormalCloud_t);
      pcl::io::loadPolygonFile(segfile, *mesh);
      geom_utils.convertMeshToPointsNormals(mesh, cloud, normals);
      all_clouds.push_back(cloud);
      all_normals.push_back(normals);
      scene_ids.push_back(i);

      ++segment_id;
      segfile = utils.getSegmentPath(i, segment_id);
    }

    // Load in the measures for the .ply files and stack them as rows into one
    // big matrix
    Eigen::MatrixXf measuremat;
    std::string measurepath = utils.getMeasurePath(i);
    eigen_extensions::loadASCII(measurepath, &measuremat);
    measures.conservativeResize(measures.rows() + measuremat.rows(),
                                measuremat.cols());
    measures.block(measures.rows() - measuremat.rows(), 0, measuremat.rows(),
                   measuremat.cols()) = measuremat;  // insert it on bottom

    printf("Loaded scene %zu/%zu with %zu segments...\n", i, kMaxSceneID,
           segment_id);
    number_of_segments += segment_id;
    number_of_scenes++;
  }
  printf(
      "Number of segments: %zu from %zu scenes. Size of measures matrix: %d x "
      "%d\n",
      number_of_segments, number_of_scenes, (int)measures.rows(),
      (int)measures.cols());
  assert(number_of_segments == ((int)measures.rows()));

  // Normalize the measures for all segments.
  printf("Normalizing measures...\n");
  Eigen::MatrixXf mm(measures);
  Eigen::VectorXf mmean = mm.colwise().sum() / mm.rows();
  mm.rowwise() -= mmean.transpose();
  Eigen::MatrixXf mstd = (mm.colwise().squaredNorm() / mm.rows()).cwiseSqrt();

  // Normalize the data.
  Eigen::MatrixXf measuresNormalized(measures);
  measuresNormalized.rowwise() -= mmean.transpose();
  measuresNormalized = measuresNormalized.cwiseQuotient(
      mstd.replicate(measuresNormalized.rows(), 1));

  // Get spatial extents of all segments.
  printf("finding spatial extents of all segments...\n");
  Eigen::MatrixXf V(number_of_segments, 3);
  for (size_t i = 0u; i < number_of_segments; ++i) {
    Eigen::MatrixXf ptsmap = all_clouds[i]->getMatrixXfMap();
    Eigen::VectorXf mins = ptsmap.rowwise().minCoeff();
    Eigen::VectorXf maxes = ptsmap.rowwise().maxCoeff();
    Eigen::VectorXf ranges = maxes - mins;
    V.row(i) = ranges.head(3);  // save the vx, vy, vz
  }

  // start the visualizer
  VisWrapper v;
  v.vis_.setBackgroundColor(0.5, 0.5, 0.5);
  v.vis_.setCameraPosition(0, 0, 0, 0.3, 0.3, 0.3, 0, -1, 0);

  printf("computing co-occurrence score\n");
  Eigen::VectorXf score(number_of_segments);
  for (size_t i = 0u; i < number_of_segments; ++i) {
    float mult = kMultply;
    Eigen::VectorXf vthr = V.row(i) * mult;
    std::vector<fipair> x;
    while (x.size() < kNumberOfNeigbors) {
      x.clear();
      for (size_t k = 0u; k < number_of_segments; ++k) {
        if (scene_ids[i] == scene_ids[k]) continue;
        if (((V.row(i) - V.row(k)).transpose().array().abs() > vthr.array())
                .any())
          continue;
        float dd = (measuresNormalized.row(i).tail(4) -
                    measuresNormalized.row(k).tail(4))
                       .array()
                       .square()
                       .sum();
        x.push_back(fipair(dd, k));
      }
      mult += 0.02f;
      vthr = V.row(i) * mult;
    }
    std::sort(x.begin(), x.end(), object_discovery::Utils::fiComparatorAscend);
    float coscore = 0.0f;
    size_t k = 0u;
    int thisscene = scene_ids[i];
    size_t numn = 0u;
    std::vector<Cloud_t::Ptr> toshow;
    std::vector<NormalCloud_t::Ptr> toshown;
    toshow.push_back(all_clouds[i]);
    toshown.push_back(all_normals[i]);
    while (numn < kNumberOfNeigbors && numn < x.size()) {
      coscore += x[k].first;
      if (display == 1) {
        printf("adding %f to coscore!\n", x[k].first);
      }
      toshow.push_back(all_clouds[x[k].second]);
      toshown.push_back(all_normals[x[k].second]);

      ++k;
      ++numn;
    }
    coscore /= numn;
    printf(" SUM=%f based on %zu measurements and extent %f for %zu/%zu\n",
           coscore, numn, mult, i, number_of_segments);
    if (display == 1) {
      geom_utils.showCloudsGridOriented(v, toshow, toshown);
      while (v.waitKey() != 'q') continue;
    }
    score(i) = -coscore;
  }

  // Visualize the final ordering.
  if (display == 2) {
    std::vector<fipair> objv(number_of_segments);
    for (size_t i = 0u; i < number_of_segments; ++i) {
      objv[i] = fipair(score(i), i);
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
    geom_utils.showCloudsGridOriented(v, cs, ns);
    while (v.waitKey() != 'q') {
      continue;
    }
  }

  // Save the measures that include co-occurrence as last column in a file
  // "segmentMeasures.etc.txt".
  std::string measuresPath =
      utils.ROOT_PATH + std::string("segmentMeasures.eig.txt");
  measures.conservativeResize(measures.rows(), measures.cols() + 1);
  measures.col(measures.cols() - 1) = score;

  // Normalize before saving.
  Eigen::VectorXf mfMean = measures.colwise().sum() / measures.rows();
  measures.rowwise() -= mfMean.transpose();
  Eigen::MatrixXf mfStd =
      (measures.colwise().squaredNorm() / measures.rows()).cwiseSqrt();
  measures = measures.cwiseQuotient(mfStd.replicate(measures.rows(), 1));

  eigen_extensions::saveASCII(measures, measuresPath);
  cout << "Saving final measures to " << measuresPath << endl;

  return (0);
}
