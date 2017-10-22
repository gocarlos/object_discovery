/*
Copyright (C) 2006 Pedro Felzenszwalb

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
*/

#include "object_discovery/segment_graph.h"

#include <algorithm>
#include <cmath>

#include "object_discovery/disjoint_set.h"

bool operator<(const edge& a, const edge& b) { return a.w < b.w; }

universe* segment_graph(const int num_vertices, const int num_edges,
                        const float k_threshold, edge* edges) {
  // Sort edges by weight.
  std::sort(edges, edges + num_edges);

  // Make a disjoint-set forest.
  universe* u = new universe(num_vertices);

  // Init thresholds.
  float* threshold = new float[num_vertices];
  for (size_t i = 0u; i < num_vertices; ++i) {
    threshold[i] = THRESHOLD(1, k_threshold);
  }

  // For each edge, in non-decreasing weight order.
  for (size_t i = 0u; i < num_edges; ++i) {
    edge* pedge = &edges[i];

    // Components connected by this edge.
    int a = u->find(pedge->a);
    int b = u->find(pedge->b);
    if (a != b) {
      if ((pedge->w <= threshold[a]) && (pedge->w <= threshold[b])) {
        u->join(a, b);
        a = u->find(a);
        threshold[a] = pedge->w + THRESHOLD(u->size(a), k_threshold);
      }
    }
  }

  // Free up
  delete threshold;
  return u;
}
