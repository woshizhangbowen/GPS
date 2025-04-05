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

#ifndef COLMAP_SRC_BASE_DATABASE_CACHE_H_
#define COLMAP_SRC_BASE_DATABASE_CACHE_H_

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>

#include "base/camera.h"
#include "base/camera_models.h"
#include "base/correspondence_graph.h"
#include "base/database.h"
#include "base/image.h"
#include "util/alignment.h"
#include "util/types.h"

namespace colmap {

// A class that caches the contents of the database in memory, used to quickly  // 一个将数据库内容缓存在内存中的类，
// create new reconstruction instances when multiple models are reconstructed.  // 用于在重建多个模型时快速创建新的重建实例
class DatabaseCache {
 public:
  DatabaseCache();

  // Get number of objects.  // 获取缓存中对象数量
  inline size_t NumCameras() const;
  inline size_t NumImages() const;

  // Get specific objects.  // 根据camera_id和image_id获取特定对象
  inline class Camera& Camera(const camera_t camera_id);
  inline const class Camera& Camera(const camera_t camera_id) const;
  inline class Image& Image(const image_t image_id);
  inline const class Image& Image(const image_t image_id) const;

  // Get all objects.
  inline const EIGEN_STL_UMAP(camera_t, class Camera) & Cameras() const;  // alignment.h中定义的函数，返回一个常量引用，指向存储所有相机的容器 cameras_
  inline const EIGEN_STL_UMAP(image_t, class Image) & Images() const;  // 容器的类型是 EIGEN_STL_UMAP(image_t, class Image),基于 std::unordered_map 的自定义类型

  // Check whether specific object exists.  // 检查缓存中是否存在特定的相机或图像
  inline bool ExistsCamera(const camera_t camera_id) const;
  inline bool ExistsImage(const image_t image_id) const;

  // Get reference to correspondence graph.  // 获取对应图
  inline const class CorrespondenceGraph& CorrespondenceGraph() const;  // 返回缓存中correspondence_graph_ 的常量引用

  // Manually add data to cache.  // 手动添加数据到缓存中
  void AddCamera(const class Camera& camera);
  void AddImage(const class Image& image);

  // Load cameras, images, features, and matches from database.
  //
  // @param database              Source database from which to load data.
  // @param min_num_matches       Only load image pairs with a minimum number
  //                              of matches.                               // 仅加载匹配数大于等于该值的图像对!!!!!!
  // @param ignore_watermarks     Whether to ignore watermark image pairs.  // 忽略水印图像对
  // @param image_names           Whether to use only load the data for a subset  // 一个字符串集合，指定要加载的图像子集
  //                              of the images. All images are used if empty.   // 如果为空，则加载所有图像
  void Load(const Database& database, const size_t min_num_matches,
            const bool ignore_watermarks,
            const std::unordered_set<std::string>& image_names,
             std::string &speed_path,std::string &feature_prior_depth_path);

  // Find specific image by name. Note that this uses linear search.
  const class Image* FindImageWithName(const std::string& name) const;

  bool ReadKeyPointsDepthValue(const std::string &feature_prior_depth_path,
                               const std::string &name,
                               std::unordered_map<int, double>& KeyDepth);

 private:
  class CorrespondenceGraph correspondence_graph_;


  EIGEN_STL_UMAP(camera_t, class Camera) cameras_;
  EIGEN_STL_UMAP(image_t, class Image) images_;
};

////////////////////////////////////////////////////////////////////////////////
// Implementation
////////////////////////////////////////////////////////////////////////////////

size_t DatabaseCache::NumCameras() const { return cameras_.size(); }
size_t DatabaseCache::NumImages() const { return images_.size(); }

class Camera& DatabaseCache::Camera(const camera_t camera_id) {
  return cameras_.at(camera_id);
}

const class Camera& DatabaseCache::Camera(const camera_t camera_id) const {
  return cameras_.at(camera_id);
}

class Image& DatabaseCache::Image(const image_t image_id) {
  return images_.at(image_id);
}

const class Image& DatabaseCache::Image(const image_t image_id) const {
  return images_.at(image_id);
}

const EIGEN_STL_UMAP(camera_t, class Camera) & DatabaseCache::Cameras() const {
  return cameras_;
}

const EIGEN_STL_UMAP(image_t, class Image) & DatabaseCache::Images() const {
  return images_;
}

bool DatabaseCache::ExistsCamera(const camera_t camera_id) const {
  return cameras_.find(camera_id) != cameras_.end();
}

bool DatabaseCache::ExistsImage(const image_t image_id) const {
  return images_.find(image_id) != images_.end();
}

inline const class CorrespondenceGraph& DatabaseCache::CorrespondenceGraph()
    const {
  return correspondence_graph_;
}

}  // namespace colmap

#endif  // COLMAP_SRC_BASE_DATABASE_CACHE_H_
