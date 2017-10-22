
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <string>
#include <vector>

#include <pcl/io/vtk_lib_io.h>
#include <pcl/ros/conversions.h>

#include "object_discovery/measures.h"
#include "object_discovery/utils.h"

#include "object_discovery/segmenter.hpp"

// Take a scene id and segment the scene to its object hypotheses
//  outputs .ply files for object hypotheses and measures.eig.txt file that
//  contains the objectness measures
int main(int argc, char** argv) {
  srand(1);

  object_discovery::Segmenter segmenter;
  object_discovery::Utils utils;
  object_discovery::Measures measure;

  assert(argc > 1);
  int sceneid = atoi(argv[1]);
  std::string plyfile = utils.getScenePath(sceneid);
  if (!utils.file_exists(plyfile)) {
    cout << "EEEeee, file " << plyfile << " does not exist. Quitting." << endl;
    return (0);
  }

  int dosave = 1;
  if (argc > 2) dosave = atoi(argv[2]);

  int DISPLAY = 1;
  if (argc > 3) DISPLAY = atoi(argv[3]);

  // Segmentation parameters
  std::vector<float> KTHRS;
  KTHRS.push_back(0.7);
  KTHRS.push_back(0.8);
  KTHRS.push_back(0.9);
  KTHRS.push_back(1.0);
  KTHRS.push_back(1.1);
  KTHRS.push_back(1.2);
  KTHRS.push_back(1.5);
  KTHRS.push_back(1.7);
  KTHRS.push_back(2.0);
  KTHRS.push_back(3.0);
  KTHRS.push_back(5.0);

  segmenter.params.KTHRS = KTHRS;
  segmenter.params.MINSEGSIZE = 1000;
  segmenter.params.MINPTS = 500;
  segmenter.params.MAXPTS = 50000;
  segmenter.params.THINTHR = 0.01;
  segmenter.params.FLATTHR = 0.005;
  segmenter.params.MINLENGTH = 0.05;
  segmenter.params.MAXLENGTH = 0.80;
  segmenter.params.SYMMETRY_MEASURE_CLOUD_NORMALS_TRADEOFF = 0.3;
  segmenter.params.LOCAL_CONVX_MEASURE_NNRADIUS = 0.0075;
  segmenter.params.SMOOTHNESS_MEASURE_NNRADIUS = 0.01;
  segmenter.params.SMOOTHNESS_MEASURE_NUMBINS = 8;
  segmenter.params.IOUTHR = 0.8;

  MAKECLOCK;

  // Load the geometry from .ply and extract the mesh, cloud, normals
  TIC;
  Mesh_t::Ptr sceneMesh(new Mesh_t);
  std::cout << "Loading file " << plyfile << endl;
  pcl::io::loadPolygonFile(plyfile, *sceneMesh);
  TOC("loaded geometry");

  segmenter.segmentMesh(sceneMesh);

  printf("done!\n");
  return (0);
}
