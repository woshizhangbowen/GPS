// Copyright (c) 2018, ETH Zurich and UNC Chapel Hill.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//     * Neither the name of ETH Zurich and UNC Chapel Hill nor the names of
//       its contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: Johannes L. Schoenberger (jsch-at-demuc-dot-de)

#include "base/scene_clustering.h"

#include <set>

#include "base/database.h"
#include "base/graph_cut.h"
#include "util/random.h"

namespace colmap {

bool SceneClustering::Options::Check() const {
  CHECK_OPTION_GT(branching, 0);
  CHECK_OPTION_GE(image_overlap, 0);
  return true;
}

SceneClustering::SceneClustering(const Options& options) : options_(options) {
  CHECK(options_.Check());
}

void SceneClustering::Partition(
    const std::vector<std::pair<image_t, image_t>>& image_pairs,
    const std::vector<int>& num_inliers) {
  CHECK(!root_cluster_);
  CHECK_EQ(image_pairs.size(), num_inliers.size());
  // 边是image_id1 ,image_id2
  //edges: image_id1--image_id2

  std::set<image_t> image_ids;
  std::vector<std::pair<int, int>> edges;
  edges.reserve(image_pairs.size());
  for (const auto& image_pair : image_pairs) {
    image_ids.insert(image_pair.first);
    image_ids.insert(image_pair.second);
    edges.emplace_back(image_pair.first, image_pair.second);
  }

  root_cluster_.reset(new Cluster());
  root_cluster_->image_ids.insert(root_cluster_->image_ids.end(),
                                  image_ids.begin(), image_ids.end());
  PartitionCluster(edges, num_inliers, root_cluster_.get());
  // 内点个数作为权重
}

void SceneClustering::PartitionCluster(
    const std::vector<std::pair<int, int>>& edges,
    const std::vector<int>& weights, Cluster* cluster) {
  CHECK_EQ(edges.size(), weights.size());

  // If the cluster is small enough, we return from the recursive clustering.
  if (edges.size() == 0 ||
      cluster->image_ids.size() <=
          static_cast<size_t>(options_.leaf_max_num_images)) {
    return;
  }

  // Partition the cluster using a normalized cut on the scene graph.
  // 使用N-CUT 来聚类,branching 是子集个数
  const auto labels =
      ComputeNormalizedMinGraphCut(edges, weights, options_.branching);

  // Assign the images to the clustered child clusters.
  cluster->child_clusters.resize(options_.branching);
  for (const auto image_id : cluster->image_ids) {
    if (labels.count(image_id)) {
      auto& child_cluster = cluster->child_clusters.at(labels.at(image_id));
      child_cluster.image_ids.push_back(image_id);
    }
  }

  // Collect the edges based on whether they are inter or intra child clusters.
  std::vector<std::vector<std::pair<int, int>>> child_edges(options_.branching);
  std::vector<std::vector<int>> child_weights(options_.branching);
  std::vector<std::vector<std::pair<std::pair<int, int>, int>>>
      overlapping_edges(options_.branching);
  for (size_t i = 0; i < edges.size(); ++i) {
    const int label1 = labels.at(edges[i].first);
    const int label2 = labels.at(edges[i].second);
    if (label1 == label2) {
      child_edges.at(label1).push_back(edges[i]);
      child_weights.at(label1).push_back(weights[i]);
    } else {
      overlapping_edges.at(label1).emplace_back(edges[i], weights[i]);
      overlapping_edges.at(label2).emplace_back(edges[i], weights[i]);
    }
  }

  // Recursively partition all the child clusters.
  for (int i = 0; i < options_.branching; ++i) {
    PartitionCluster(child_edges[i], child_weights[i],
                     &cluster->child_clusters[i]);
  }

  if (options_.image_overlap > 0) {
    for (int i = 0; i < options_.branching; ++i) {
      // Sort the overlapping edges by the number of inlier matches, such
      // that we add overlapping images with many common observations.
      std::sort(overlapping_edges[i].begin(), overlapping_edges[i].end(),
                [](const std::pair<std::pair<int, int>, int>& edge1,
                   const std::pair<std::pair<int, int>, int>& edge2) {
                  return edge1.second > edge2.second;
                });

      // Select overlapping edges at random and add image to cluster.
      std::set<int> overlapping_image_ids;
      for (const auto& edge : overlapping_edges[i]) {
        if (labels.at(edge.first.first) == i) {
          overlapping_image_ids.insert(edge.first.second);
        } else {
          overlapping_image_ids.insert(edge.first.first);
        }
        if (overlapping_image_ids.size() >=
            static_cast<size_t>(options_.image_overlap)) {
          break;
        }
      }

      // Recursively append the overlapping images to cluster and its children.
      std::function<void(Cluster*)> InsertOverlappingImageIds =
          [&](Cluster* cluster) {
            cluster->image_ids.insert(cluster->image_ids.end(),
                                      overlapping_image_ids.begin(),
                                      overlapping_image_ids.end());
            for (auto& child_cluster : cluster->child_clusters) {
              InsertOverlappingImageIds(&child_cluster);
            }
          };

      InsertOverlappingImageIds(&cluster->child_clusters[i]);
    }
  }
}

const SceneClustering::Cluster* SceneClustering::GetRootCluster() const {
  return root_cluster_.get();
}

std::vector<const SceneClustering::Cluster*> SceneClustering::GetLeafClusters()
    const {
  CHECK(root_cluster_);

  std::vector<const Cluster*> leaf_clusters;

  if (!root_cluster_) {
    return leaf_clusters;
  } else if (root_cluster_->child_clusters.empty()) {
    leaf_clusters.push_back(root_cluster_.get());
    return leaf_clusters;
  }

  std::vector<const Cluster*> non_leaf_clusters;
  non_leaf_clusters.push_back(root_cluster_.get());

  while (!non_leaf_clusters.empty()) {
    const auto cluster = non_leaf_clusters.back();
    non_leaf_clusters.pop_back();

    for (const auto& child_cluster : cluster->child_clusters) {
      if (child_cluster.child_clusters.empty()) {
        leaf_clusters.push_back(&child_cluster);
      } else {
        non_leaf_clusters.push_back(&child_cluster);
      }
    }
  }

  return leaf_clusters;
}

}  // namespace colmap
