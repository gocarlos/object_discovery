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

#pragma once

#include <algorithm>
#include <cmath>

#include "object_discovery/disjoint_set.h"

// Threshold function.
#define THRESHOLD(size, k_threshold) (k_threshold / size)

typedef struct {
  float w;
  int a, b;
} edge;

bool operator<(const edge& a, const edge& b);

/*!
 * Segment a graph
 *
 * @param num_vertices: number of vertices in graph.
 * @param num_edges: number of edges in graph.
 * @param edges: array of edges.
 * @param c: constant for threshold function.
 * @returns A disjoint-set forest representing the segmentation.
 */
universe* segment_graph(const int num_vertices, const int num_edges,
                        const float k_threshold, edge* edges);
