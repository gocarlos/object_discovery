#pragma once

#include <glog/logging.h>

#include "object_discovery/geom_utils.h"
#include "object_discovery/utils.h"

namespace object_discovery {
/*!
 * Objectness measures. All the methods assume that cloud is in its eigenbasis.
 * That is, it is longest along first dimension.
 */
class Measures {
 public:
  Utils utils_;
  GeomUtils geom_utils_;

  /*!
   * Compactness: ratio of surface area of the segment to surface area of its
   * smallest bounding sphere.
   */
  float scoreCompactness(const Cloud_t::Ptr cloud);

  /*!
   * Symmetry: flip the cloud along all 3 principle axes and measure overlap.
   */
  float scoreSymmetry(const Cloud_t::Ptr cloud,
                      const NormalCloud_t::Ptr normals, const float relweight);
  /*!
   * Global convexity: the convex hull should be a good approximation of the
   * segment.
   */
  float scoreGlobalConvexity(const Cloud_t::Ptr cloud);

  /*!
   * Objects are usually composed of many locally convex regions.
   */
  float scoreLocalConvexity(const Cloud_t::Ptr cloud,
                            const NormalCloud_t::Ptr ncloud,
                            const float NNradius);

  /*!
   * Projects local points to tangent plane of every point and is high when
   * there is a lot of mass in all directions around a point. i.e.
   * entropy is high of the angle distribution histogram of surrounding
   * mass in the local 2D coordinate system.
   */
  float scoreSmoothness(const Cloud_t::Ptr cloud,
                        const NormalCloud_t::Ptr ncloud, float NNradius,
                        int numbins);

  float getOverlap(const Cloud_t::Ptr cloud, const NormalCloud_t::Ptr normals,
                   Cloud_t::Ptr dest, NormalCloud_t::Ptr normdest,
                   const float relweight, float dnormalize);
};
}  // namespace object_discovery
