#pragma once

#include <dirent.h>
#include <errno.h>
#include <ros/package.h>  // ros::package::getPath
#include <stdio.h>        // included for use of rand()
#include <sys/stat.h>
#include <iostream>
#include <limits>
#include <string>
#include <utility>
#include <vector>

// PCL
#include <pcl/PolygonMesh.h>
#include <pcl/common/time.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>
#include <pcl/impl/point_types.hpp>  // PointNormal type

#define MAKECLOCK double mps_start
#define TIC mps_start = pcl::getTime()
#define TOC(a)                       \
  printf("%s in %f seconds \n", (a), \
         static_cast<double>(pcl::getTime() - mps_start))

#define MAXX(x, y) ((x) > (y) ? (x) : (y))
#define MINN(x, y) ((x) < (y) ? (x) : (y))
typedef std::pair<float, int> fipair;

// In case users don't have color, set USE_COLOR to false.
#define USE_COLOR 1

typedef pcl::PointXYZ PointBW_t;
typedef pcl::PointXYZRGB PointColor_t;

#ifdef USE_COLOR
typedef PointColor_t Point_t;
#else
typedef PointBW_t Point_t;
#endif

typedef pcl::PointCloud<PointColor_t> CloudColor_t;
typedef pcl::PointCloud<PointBW_t> CloudBW_t;

typedef pcl::PointCloud<Point_t> Cloud_t;
typedef pcl::PolygonMesh Mesh_t;
typedef pcl::search::KdTree<Point_t> Tree_t;

typedef pcl::PointXYZI KeyPoint_t;
typedef pcl::PointCloud<KeyPoint_t> KeyCloud_t;

typedef pcl::Normal Normal_t;
typedef pcl::PointCloud<Normal_t> NormalCloud_t;

namespace object_discovery {
class Utils {
 public:
  Utils();
  std::string package_path;
  std::string ROOT_PATH;

  std::string getScenePath(int sceneid);

  std::string getMeasurePath(int sceneid);

  std::string getSegmentPath(int sceneid, int segid);

  // fills files[] vector with all filenames found in 'dir', except for '.' and
  // '..'
  int listdir(const std::string dir, std::vector<std::string>& files);

  float randf(float a, float b);

  bool file_exists(std::string file);

  static bool fiComparatorDescend(const std::pair<float, int>& l,
                                  const std::pair<float, int>& r);
  static bool fiComparatorAscend(const std::pair<float, int>& l,
                                 const std::pair<float, int>& r);
};
}  // namespace object_discovery
