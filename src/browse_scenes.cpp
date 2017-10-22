
#include <stdio.h>
#include <string>

#include <pcl/io/vtk_lib_io.h>

#include "object_discovery/geom_utils.h"
#include "object_discovery/utils.h"
#include "object_discovery/vis_wrapper.h"

// Simple script for exploring the database of scenes in scenes/ folder
int main(int argc, char** argv) {
  int sceneid = 0;
  if (argc > 1) sceneid = atoi(argv[1]);

  object_discovery::Utils lib_utils;
  object_discovery::GeomUtils geom_utils;
  printf("USAGE:");
  printf("w: go a scene up, s: go a scene down, q: quit\n");
  VisWrapper v;
  while (true) {
    // Load mesh
    std::string plyfile = lib_utils.getScenePath(sceneid);
    Mesh_t::Ptr mesh(new Mesh_t);
    pcl::io::loadPolygonFile(plyfile, *mesh);

    // Display it
    printf("Displaying scene %d, filename %s\n", sceneid, plyfile.c_str());
    Cloud_t::Ptr sceneCloud(new Cloud_t);
    NormalCloud_t::Ptr sceneNormals(new NormalCloud_t);
    geom_utils.convertMeshToPointsNormals(mesh, sceneCloud, sceneNormals);
    // v.showMesh(mesh);
    v.showCloud(sceneCloud);

    while (v.waitKey() != 'q') continue;
  }

  return (0);
}
