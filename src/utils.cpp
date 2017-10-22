#include "object_discovery/utils.h"

#include <string>
#include <vector>

namespace object_discovery {
// OPTIMIZE(gocarlos): maybe one should not be using ROS here: hardcode some
// path, again?
Utils::Utils() {
  package_path = ros::package::getPath("object_discovery");
  ROOT_PATH = package_path + "/";  // put a slash at the end of the path name.
}

std::string Utils::getScenePath(int sceneid) {
  char sbuf[100];
  sprintf(sbuf, "scenes/scene%04d.ply", sceneid);
  return ROOT_PATH + std::string(sbuf);
}

std::string Utils::getMeasurePath(int sceneid) {
  char sbuf[100];
  sprintf(sbuf, "measures/measure-s%04d.eig.txt", sceneid);

  return ROOT_PATH + std::string(sbuf);
}

std::string Utils::getSegmentPath(int sceneid, int segid) {
  char sbuf[100];
  sprintf(sbuf, "segments/segment-s%04d-g%04d.ply", sceneid, segid);
  return ROOT_PATH + std::string(sbuf);
}
// fills files[] vector with all filenames found in 'dir', except for '.' and
// '..'
int Utils::listdir(const std::string dir, std::vector<std::string>& files) {
  DIR* dp;
  struct dirent* dirp;
  if ((dp = opendir(dir.c_str())) == NULL) {
    std::cout << "Error(" << errno << ") opening " << dir << std::endl;
    return errno;
  }
  while ((dirp = readdir(dp)) != NULL) {
    std::string fname = std::string(dirp->d_name);
    if (fname == "." || fname == "..") continue;
    files.push_back(fname);
  }
  closedir(dp);
  return 0;
}

float Utils::randf(float a, float b) {
  return (static_cast<float>(rand()) / static_cast<float>(RAND_MAX)) * (b - a) +
         a;
}

bool Utils::fiComparatorDescend(const fipair& l, const fipair& r) {
  return l.first > r.first;
}
bool Utils::fiComparatorAscend(const fipair& l, const fipair& r) {
  return l.first < r.first;
}

bool Utils::file_exists(std::string file) {
  bool exists = false;
  struct stat sbuf;
  if (stat(file.c_str(), &sbuf) != -1) exists = true;
  return exists;
}
}  // namespace object_discovery
