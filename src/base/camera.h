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

#ifndef COLMAP_SRC_BASE_CAMERA_H_
#define COLMAP_SRC_BASE_CAMERA_H_

#include <vector>

#include "util/types.h"

namespace colmap {

// Camera class that holds the intrinsic parameters. Cameras may be shared   相机类，用于保存内在参数。相机可以在多张图像之间共享，
// between multiple images, e.g., if the same "physical" camera took multiple  例如，如果同一台“物理”相机使用完全相同的镜头和内在参数（如焦距等）拍摄了多张照片。
// pictures with the exact same lens and intrinsics (focal length, etc.).
// This class has a specific distortion model defined by a camera model class.  该类具有由相机模型类定义的具体畸变模型。
class Camera {
 public:
  Camera();

  // Access the unique identifier of the camera.  访问相机的唯一标识符。
  inline camera_t CameraId() const;  // 获取
  inline void SetCameraId(const camera_t camera_id);  // 设置

  // Access the camera model.  访问相机模型的方法是通过 ModelId() 和 ModelName() 
  inline int ModelId() const;
  std::string ModelName() const;
  void SetModelId(const int model_id);
  void SetModelIdFromName(const std::string& model_name);

  // Access dimensions of the camera sensor.  访问相机传感器尺寸的方法
  inline size_t Width() const;
  inline size_t Height() const;
  inline void SetWidth(const size_t width);
  inline void SetHeight(const size_t height);

  // Access focal length parameters.
  double MeanFocalLength() const;  // 获取平均焦距
  double FocalLength() const;  // 获取焦距
  double FocalLengthX() const;  // 获取X方向焦距
  double FocalLengthY() const;  // 获取Y方向焦距
  void SetFocalLength(const double focal_length);  // 设置焦距
  void SetFocalLengthX(const double focal_length_x);  // 设置X方向焦距
  void SetFocalLengthY(const double focal_length_y);  

  // Check if camera has prior focal length.  判断相机是否有焦距的先验值
  inline bool HasPriorFocalLength() const;   // 判断相机是否有焦距的先验值
  inline void SetPriorFocalLength(const bool prior);

  // Access principal point parameters. Only works if there are two   主点参数：？？？？
  // principal point parameters.    访问主点参数的方法，但这些方法仅在相机模型中有两个主点参数时有效。
  double PrincipalPointX() const;
  double PrincipalPointY() const;
  void SetPrincipalPointX(const double ppx);
  void SetPrincipalPointY(const double ppy);

  // Get the indices of the parameter groups in the parameter vector.  获取参数组索引
  const std::vector<size_t>& FocalLengthIdxs() const;  // 获取焦距参数的索引
  const std::vector<size_t>& PrincipalPointIdxs() const;  // 获取主点参数的索引
  const std::vector<size_t>& ExtraParamsIdxs() const;  // 获取其他参数的索引

  // Get intrinsic calibration matrix composed from focal length and principal
  // point parameters, excluding distortion parameters.  CalibrationMatrix 方法用于获取由焦距和主点参数组成的内参校准矩阵，不包括畸变参数。
  Eigen::Matrix3d CalibrationMatrix() const;

  // Get human-readable information about the parameter vector ordering.  获取有关参数向量排序人类可读信息的信息。
  std::string ParamsInfo() const;

  // Access the raw parameter vector.    访问相机的原始参数向量
  inline size_t NumParams() const;
  inline const std::vector<double>& Params() const;  // 获取常量引用，用于读取参数向量。
  inline std::vector<double>& Params();  // 获取非常量引用，用于修改参数向量。
  inline double Params(const size_t idx) const;  // 获取参数向量中指定索引位置的参数值。
  inline double& Params(const size_t idx);  // 获取非常量引用，用于修改参数向量中指定索引位置的参数值。
  inline const double* ParamsData() const;  // 获取参数向量中的数据指针。用于读取参数向量。
  inline double* ParamsData();   //获取指向参数向量数据的非常量指针，用于修改参数向量。
  inline void SetParams(const std::vector<double>& params);  // 设置参数向量

  // Concatenate parameters as comma-separated list.  拼接参数为逗号分隔列表。
  std::string ParamsToString() const;  

  // Set camera parameters from comma-separated list.  从逗号分隔列表中设置相机参数。
  bool SetParamsFromString(const std::string& string);

  // Check whether parameters are valid, i.e. the parameter vector has  检查相机参数是否有效
  // the correct dimensions that match the specified camera model.  有效性检查的标准是参数向量的维度是否与指定的相机模型相匹配
  bool VerifyParams() const;

  // Check whether camera has bogus parameters.  检查相机是否有 bogus 参数（不符合预期或不合理的相机参数）（比如焦距和额外参数）
  bool HasBogusParams(const double min_focal_length_ratio,
                      const double max_focal_length_ratio,
                      const double max_extra_param) const;

  // Initialize parameters for given camera model and focal length, and set
  // the principal point to be the image center.
  void InitializeWithId(const int model_id, const double focal_length,
                        const size_t width, const size_t height);
  void InitializeWithName(const std::string& model_name,
                          const double focal_length, const size_t width,
                          const size_t height);

  // Project point in image plane to world / infinity.
  Eigen::Vector2d ImageToWorld(const Eigen::Vector2d& image_point) const;

  // Convert pixel threshold in image plane to world space.
  double ImageToWorldThreshold(const double threshold) const;

  // Project point from world / infinity to image plane.
  Eigen::Vector2d WorldToImage(const Eigen::Vector2d& world_point) const;

  // Rescale camera dimensions and accordingly the focal length and
  // and the principal point.
  void Rescale(const double scale);
  void Rescale(const size_t width, const size_t height);

 private:
  // The unique identifier of the camera. If the identifier is not specified
  // it is set to `kInvalidCameraId`.
  camera_t camera_id_;

  // The identifier of the camera model. If the camera model is not specified
  // the identifier is `kInvalidCameraModelId`.
  int model_id_;

  // The dimensions of the image, 0 if not initialized.
  size_t width_;
  size_t height_;

  // The focal length, principal point, and extra parameters. If the camera
  // model is not specified, this vector is empty.
  std::vector<double> params_;

  // Whether there is a safe prior for the focal length,
  // e.g. manually provided or extracted from EXIF
  bool prior_focal_length_;
};

////////////////////////////////////////////////////////////////////////////////
// Implementation
////////////////////////////////////////////////////////////////////////////////

camera_t Camera::CameraId() const { return camera_id_; }

void Camera::SetCameraId(const camera_t camera_id) { camera_id_ = camera_id; }

int Camera::ModelId() const { return model_id_; }

size_t Camera::Width() const { return width_; }

size_t Camera::Height() const { return height_; }

void Camera::SetWidth(const size_t width) { width_ = width; }

void Camera::SetHeight(const size_t height) { height_ = height; }

bool Camera::HasPriorFocalLength() const { return prior_focal_length_; }

void Camera::SetPriorFocalLength(const bool prior) {
  prior_focal_length_ = prior;
}

size_t Camera::NumParams() const { return params_.size(); }

const std::vector<double>& Camera::Params() const { return params_; }

std::vector<double>& Camera::Params() { return params_; }

double Camera::Params(const size_t idx) const { return params_[idx]; }

double& Camera::Params(const size_t idx) { return params_[idx]; }

const double* Camera::ParamsData() const { return params_.data(); }

double* Camera::ParamsData() { return params_.data(); }

void Camera::SetParams(const std::vector<double>& params) { params_ = params; }

}  // namespace colmap

#endif  // COLMAP_SRC_BASE_CAMERA_H_
