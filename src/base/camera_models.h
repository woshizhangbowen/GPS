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

#ifndef COLMAP_SRC_BASE_CAMERA_MODELS_H_
#define COLMAP_SRC_BASE_CAMERA_MODELS_H_

#include <cfloat>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <ceres/ceres.h>

namespace colmap {

// This file defines several different camera models and arbitrary new camera
// models can be added by the following steps:此文件定义了几种不同的相机模型，通过以下步骤可以添加任意新的相机模型：
//
//  1. Add a new struct in this file which implements all the necessary methods. 在此文件中添加一个新的结构体，并实现所有必要的方法。
//  2. Define an unique model_name and model_id for the camera model. 为该相机模型定义一个唯一的 model_name 和 model_id
//  3. Add camera model to `CAMERA_MODEL_CASES` macro in this file. 在此文件中将相机模型添加到 CAMERA_MODEL_CASES 宏中
//  4. Add new template specialization of test case for camera model to 向 camera_models_test.cc 中添加针对该相机模型的新模板特化测试用例
//     `camera_models_test.cc`.
//
// A camera model can have three different types of camera parameters: focal 相机模型可以有三种不同类型的相机参数：焦距、主点和额外参数（畸变参数）
// length, principal point, extra parameters (distortion parameters). The    参数数组被划分为不同的组。这样我们可以在捆绑调整（bundle adjustment）期间
// parameter array is split into different groups, so that we can enable or   启用或禁用对各个组的参数进行优化。 如何正确访问这些参数由相机模型自行决定
// disable the refinement of the individual groups during bundle adjustment. It（可以以任意方式处理）——这些参数不会从外部访问。
// is up to the camera model to access the parameters correctly (it is free to
// do so in an arbitrary manner) - the parameters are not accessed from outside.
//
// A camera model must have the following methods:                              相机模型必须具有以下方法
//
//  - `WorldToImage`: transform normalized camera coordinates to image          WorldToImage   将归一化相机坐标转换为图像坐标（ImageToWorld 的逆操作）
//    coordinates (the inverse of `ImageToWorld`). Assumes that the world        假设世界坐标的输入形式为 (u, v, 1)
//    coordinates are given as (u, v, 1).
//  - `ImageToWorld`: transform image coordinates to normalized camera          ImageToWorld   输出的世界坐标形式为 (u, v, 1)。
//    coordinates (the inverse of `WorldToImage`). Produces world coordinates
//    as (u, v, 1).
//  - `ImageToWorldThreshold`: transform a threshold given in pixels to        ImageToWorldThreshold：将像素空间的阈值转换为归一化相机坐标空间的阈值，让误差判断在不同坐标系下保持物理意义的一致性
//    normalized units (e.g. useful for reprojection error thresholds).        例如，用于重投影误差阈值时很有用（重投影误差必须小于 5 像素，这时需要将 "5 像素" 转换为归一化单位）公式：pixel_threshold / focal_length
//                                                                            （因为坐标是用归一化单位（例如：以焦距 f 为尺度的无单位值）表示的） 如果焦距是 1000 像素，那么 5 像素的误差相当于归一化空间的 5/1000 = 0.005 单位 
// Whenever you specify the camera parameters in a list, they must appear       // 在列表中指定相机参数时，它们的顺序必须与定义的模型结构体中访问这些参数的顺序完全一致。
// exactly in the order as they are accessed in the defined model struct.
//
// The camera models follow the convention that the upper left image corner has    相机模型遵循以下约定：图像左上角的坐标为 (0, 0)，右下角的坐标为 (width, height)，
// the coordinate (0, 0), the lower right corner (width, height), i.e. that
// the upper left pixel center has coordinate (0.5, 0.5) and the lower right       左上角像素中心的坐标为 (0.5, 0.5)，右下角像素中心的坐标为 (width - 0.5, height - 0.5)
// pixel center has the coordinate (width - 0.5, height - 0.5).

static const int kInvalidCameraModelId = -1;

#ifndef CAMERA_MODEL_DEFINITIONS
#define CAMERA_MODEL_DEFINITIONS(model_id_value, model_name_value,             \
                                 num_params_value)                             \
  static const int kModelId = model_id_value;                                  \
  static const size_t kNumParams = num_params_value;                           \
  static const int model_id;                                                   \
  static const std::string model_name;                                         \
  static const size_t num_params;                                              \
  static const std::string params_info;                                        \
  static const std::vector<size_t> focal_length_idxs;                          \
  static const std::vector<size_t> principal_point_idxs;                       \
  static const std::vector<size_t> extra_params_idxs;                          \
                                                                               \
  static inline int InitializeModelId() { return model_id_value; };            \
  static inline std::string InitializeModelName() {                            \
    return model_name_value;                                                   \
  };                                                                           \
  static inline size_t InitializeNumParams() { return num_params_value; };     \
  static inline std::string InitializeParamsInfo();                            \
  static inline std::vector<size_t> InitializeFocalLengthIdxs();               \
  static inline std::vector<size_t> InitializePrincipalPointIdxs();            \
  static inline std::vector<size_t> InitializeExtraParamsIdxs();               \
  static inline std::vector<double> InitializeParams(                          \
      const double focal_length, const size_t width, const size_t height);     \
                                                                               \
  template <typename T>                                                        \
  static void WorldToImage(const T* params, const T u, const T v, T* x, T* y); \
  template <typename T>                                                        \
  static void ImageToWorld(const T* params, const T x, const T y, T* u, T* v); \
  template <typename T>                                                        \
  static void Distortion(const T* extra_params, const T u, const T v, T* du,   \
                         T* dv);
#endif

/*
model_id_value：相机模型id值    num_params_value：相机模型参数个数   params_info：相机模型参数信息   focal_length_idxs：存储焦距参数在参数数组中的索引，比如可能是params[0]
principal_point_idxs：存储主点坐标（光心）在参数数组中的索引，cx 和 cy 可能分别对应 params[1] 和 params[2]     extra_params_idxs：存储畸变参数在参数数组中的索引，k1, k2, k3 可能对应 params[3]、params[4] 和 params[5]
InitializeParams：根据给定的焦距 focal_length 和图像宽高 width, height 初始化参数列表，通常用于相机参数的默认初始化。
WorldToImage：将世界坐标系下的点投影到图像坐标系。params：相机模型参数数组；u, v：三维空间点的归一化坐标（未投影）；x, y：投影到图像上的二维坐标（像素坐标）
Distortion：校正或模拟镜头畸变。extra_params：畸变参数数组（例如径向畸变 k1, k2, k3）；u, v：未畸变的坐标；du, dv：畸变后的坐标增量

*/


#ifndef CAMERA_MODEL_CASES
#define CAMERA_MODEL_CASES                          \
  CAMERA_MODEL_CASE(SimplePinholeCameraModel)       \
  CAMERA_MODEL_CASE(PinholeCameraModel)             \
  CAMERA_MODEL_CASE(SimpleRadialCameraModel)        \
  CAMERA_MODEL_CASE(SimpleRadialFisheyeCameraModel) \
  CAMERA_MODEL_CASE(RadialCameraModel)              \
  CAMERA_MODEL_CASE(RadialFisheyeCameraModel)       \
  CAMERA_MODEL_CASE(OpenCVCameraModel)              \
  CAMERA_MODEL_CASE(OpenCVFisheyeCameraModel)       \
  CAMERA_MODEL_CASE(FullOpenCVCameraModel)          \
  CAMERA_MODEL_CASE(FOVCameraModel)                 \
  CAMERA_MODEL_CASE(ThinPrismFisheyeCameraModel)
#endif

#ifndef CAMERA_MODEL_SWITCH_CASES
#define CAMERA_MODEL_SWITCH_CASES         \
  CAMERA_MODEL_CASES                      \
  default:                                \
    CAMERA_MODEL_DOES_NOT_EXIST_EXCEPTION \
    break;
#endif

#define CAMERA_MODEL_DOES_NOT_EXIST_EXCEPTION \
  throw std::domain_error("Camera model does not exist");

// The "Curiously Recurring Template Pattern" (CRTP) is used here, so that we     这里使用了“奇异递归模板模式”（CRTP），以便在
// can reuse some shared functionality between all camera models -                所有相机模型之间复用一些共享功能，
// defined in the BaseCameraModel.                                                这些功能在 BaseCameraModel 中定义
template <typename CameraModel>  // CRTP 作用：让 BaseCameraModel 在继承时可以使用 CameraModel 作为派生类，从而在基类中调用派生类的方法
struct BaseCameraModel {
  template <typename T>
  static inline bool HasBogusParams(const std::vector<T>& params,
                                    const size_t width, const size_t height,
                                    const T min_focal_length_ratio,  // 焦距的最小和最大允许比例
                                    const T max_focal_length_ratio,
                                    const T max_extra_param);  // 检查相机参数是否异常, 如果参数无效，返回 true

  template <typename T>
  static inline bool HasBogusFocalLength(const std::vector<T>& params,
                                         const size_t width,
                                         const size_t height,
                                         const T min_focal_length_ratio,
                                         const T max_focal_length_ratio);  // 检查焦距

  template <typename T>
  static inline bool HasBogusPrincipalPoint(const std::vector<T>& params,
                                            const size_t width,
                                            const size_t height);  // 检查主点

  template <typename T>
  static inline bool HasBogusExtraParams(const std::vector<T>& params,
                                         const T max_extra_param);  // 检查畸变参数

  template <typename T>
  static inline T ImageToWorldThreshold(const T* params, const T threshold);  // 根据相机参数 params，将图像空间中的 threshold（阈值）转换为世界坐标空间的阈值 返回值：转换后的世界坐标阈值。

  template <typename T>
  static inline void IterativeUndistortion(const T* params, T* u, T* v);  // 将图像坐标 (u, v) 进行迭代去畸变处理，使其更加符合真实世界坐标。无返回值，直接修改 u, v 的值
};

// Simple Pinhole camera model.
//
// No Distortion is assumed. Only focal length and principal point is modeled. 假设无畸变，仅对焦距和主点进行建模
//
// Parameter list is expected in the following order:  // 模型参数列表的顺序
//
//   f, cx, cy
//
// See https://en.wikipedia.org/wiki/Pinhole_camera_model
struct SimplePinholeCameraModel
    : public BaseCameraModel<SimplePinholeCameraModel> {
  CAMERA_MODEL_DEFINITIONS(0, "SIMPLE_PINHOLE", 3)
};  // struct 定义了一种继承方式，其中 SimplePinholeCameraModel 继承自 BaseCameraModel<SimplePinholeCameraModel>.这是CRTP的典型用法。

// Pinhole camera model.
//
// No Distortion is assumed. Only focal length and principal point is modeled.
//
// Parameter list is expected in the following order:
//
//    fx, fy, cx, cy
//
// See https://en.wikipedia.org/wiki/Pinhole_camera_model
struct PinholeCameraModel : public BaseCameraModel<PinholeCameraModel> {
  CAMERA_MODEL_DEFINITIONS(1, "PINHOLE", 4)
};

// Simple camera model with one focal length and one radial distortion
// parameter.
//
// This model is similar to the camera model that VisualSfM uses with the  // 类似于 VisualSfM 使用的相机模型
// difference that the distortion here is applied to the projections and   // 不同之处在于，这里的畸变作用于投影，而不是测量值。
// not to the measurements.
//
// Parameter list is expected in the following order:
//
//    f, cx, cy, k
//
struct SimpleRadialCameraModel
    : public BaseCameraModel<SimpleRadialCameraModel> {
  CAMERA_MODEL_DEFINITIONS(2, "SIMPLE_RADIAL", 4)
};

// Simple camera model with one focal length and two radial distortion  // 两个径向畸变参数
// parameters.
//
// This model is equivalent to the camera model that Bundler uses  // 该模型等效于 Bundler 使用的相机模型
// (except for an inverse z-axis in the camera coordinate system).  //不同之处在于相机坐标系中的 z 轴方向相反
//
// Parameter list is expected in the following order:
//
//    f, cx, cy, k1, k2
//
struct RadialCameraModel : public BaseCameraModel<RadialCameraModel> {
  CAMERA_MODEL_DEFINITIONS(3, "RADIAL", 5)
};

// OpenCV camera model.
//
// Based on the pinhole camera model. Additionally models radial and  // 基于针孔相机模型，此外还对径向和切向畸变进行建模（最多包含二阶系数）
// tangential distortion (up to 2nd degree of coefficients). Not suitable for
// large radial distortions of fish-eye cameras.    // 不适用于具有大径向畸变的鱼眼相机。
//
// Parameter list is expected in the following order:
//
//    fx, fy, cx, cy, k1, k2, p1, p2
//
// See
// http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
struct OpenCVCameraModel : public BaseCameraModel<OpenCVCameraModel> {
  CAMERA_MODEL_DEFINITIONS(4, "OPENCV", 8)
};

// OpenCV fish-eye camera model.
//
// Based on the pinhole camera model. Additionally models radial and
// tangential Distortion (up to 2nd degree of coefficients). Suitable for
// large radial distortions of fish-eye cameras.
//
// Parameter list is expected in the following order:
//
//    fx, fy, cx, cy, k1, k2, k3, k4
//
// See
// http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
struct OpenCVFisheyeCameraModel
    : public BaseCameraModel<OpenCVFisheyeCameraModel> {
  CAMERA_MODEL_DEFINITIONS(5, "OPENCV_FISHEYE", 8)
};

// Full OpenCV camera model.
//
// Based on the pinhole camera model. Additionally models radial and   // 基于针孔相机模型，此外还对径向和切向畸变进行建模
// tangential Distortion.
//
// Parameter list is expected in the following order:
//
//    fx, fy, cx, cy, k1, k2, p1, p2, k3, k4, k5, k6
//
// See
// http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
struct FullOpenCVCameraModel : public BaseCameraModel<FullOpenCVCameraModel> {
  CAMERA_MODEL_DEFINITIONS(6, "FULL_OPENCV", 12)
};

// FOV camera model.
//
// Based on the pinhole camera model. Additionally models radial distortion.  基于针孔相机模型，此外还对径向畸变进行建模
// This model is for example used by Project Tango for its equidistant        例如，该模型被 Project Tango 用于其等距标定类型
// calibration type.
//
// Parameter list is expected in the following order:
//
//    fx, fy, cx, cy, omega
//
// See:  // 参考文献
// Frederic Devernay, Olivier Faugeras. Straight lines have to be straight:
// Automatic calibration and removal of distortion from scenes of structured
// environments. Machine vision and applications, 2001.
struct FOVCameraModel : public BaseCameraModel<FOVCameraModel> {
  CAMERA_MODEL_DEFINITIONS(7, "FOV", 5)

  template <typename T>
  static void Undistortion(const T* extra_params, const T u, const T v, T* du,
                           T* dv);
};

// Simple camera model with one focal length and one radial distortion
// parameter, suitable for fish-eye cameras.
//
// This model is equivalent to the OpenCVFisheyeCameraModel but has only one
// radial distortion coefficient.
//
// Parameter list is expected in the following order:
//
//    f, cx, cy, k
//
struct SimpleRadialFisheyeCameraModel
    : public BaseCameraModel<SimpleRadialFisheyeCameraModel> {
  CAMERA_MODEL_DEFINITIONS(8, "SIMPLE_RADIAL_FISHEYE", 4)
};

// Simple camera model with one focal length and two radial distortion
// parameters, suitable for fish-eye cameras.
//
// This model is equivalent to the OpenCVFisheyeCameraModel but has only two
// radial distortion coefficients.
//
// Parameter list is expected in the following order:
//
//    f, cx, cy, k1, k2
//
struct RadialFisheyeCameraModel
    : public BaseCameraModel<RadialFisheyeCameraModel> {
  CAMERA_MODEL_DEFINITIONS(9, "RADIAL_FISHEYE", 5)
};

// Camera model with radial and tangential distortion coefficients and
// additional coefficients accounting for thin-prism distortion.  具有径向和切向畸变系数的相机模型，并包含用于薄棱镜畸变的额外系数
//
// This camera model is described in
//
//    "Camera Calibration with Distortion Models and Accuracy Evaluation",   参考文献
//    J Weng et al., TPAMI, 1992.
//
// Parameter list is expected in the following order:
//
//    fx, fy, cx, cy, k1, k2, p1, p2, k3, k4, sx1, sy1
//
struct ThinPrismFisheyeCameraModel
    : public BaseCameraModel<ThinPrismFisheyeCameraModel> {
  CAMERA_MODEL_DEFINITIONS(10, "THIN_PRISM_FISHEYE", 12)
};


// Check whether camera model with given name or identifier exists.  检查是否存在具有指定名称或标识符的相机模型
bool ExistsCameraModelWithName(const std::string& model_name);   // 根据相机模型名称检查是否存在
bool ExistsCameraModelWithId(const int model_id);                // 根据相机模型标识符检查是否存在

// Convert camera name to unique camera model identifier.  // 将相机模型名称转换为唯一的相机模型标识符
//
// @param name         Unique name of camera model.
//
// @return             Unique identifier of camera model.
int CameraModelNameToId(const std::string& model_name);

// Convert camera model identifier to unique camera model name.
//
// @param model_id     Unique identifier of camera model.
//
// @return             Unique name of camera model.
std::string CameraModelIdToName(const int model_id);

// Initialize camera parameters using given image properties.  初始化相机参数，使用给定的图像属性
//
// Initializes all focal length parameters to the same given focal length and  // 初始化所有焦距参数为相同的给定焦距
// sets the principal point to the image center.  // 设置主点到图像中心
//
// @param model_id      Unique identifier of camera model.
// @param focal_length  Focal length, equal for all focal length parameters.
// @param width         Sensor width of the camera.
// @param height        Sensor height of the camera.
std::vector<double> CameraModelInitializeParams(const int model_id,
                                                const double focal_length,
                                                const size_t width,
                                                const size_t height);

// Get human-readable information about the parameter vector order.  // 获取有关参数向量顺序的人类可读信息
//
// @param model_id     Unique identifier of camera model.    相机模型的唯一标识符
std::string CameraModelParamsInfo(const int model_id);

// Get the indices of the parameter groups in the parameter vector.  // 获取参数向量中参数组的索引
//
// @param model_id     Unique identifier of camera model.
const std::vector<size_t>& CameraModelFocalLengthIdxs(const int model_id);
const std::vector<size_t>& CameraModelPrincipalPointIdxs(const int model_id);
const std::vector<size_t>& CameraModelExtraParamsIdxs(const int model_id);

// Get the total number of parameters of a camera model.
size_t CameraModelNumParams(const int model_id);

// Check whether parameters are valid, i.e. the parameter vector has
// the correct dimensions that match the specified camera model.
//
// @param model_id      Unique identifier of camera model.
// @param params        Array of camera parameters.
bool CameraModelVerifyParams(const int model_id,
                             const std::vector<double>& params);

// Check whether camera has bogus parameters.  检查相机是否有 bogus(无效的) 参数
//
// @param model_id                Unique identifier of camera model.
// @param params                  Array of camera parameters.
// @param width                   Sensor width of the camera.
// @param height                  Sensor height of the camera.
// @param min_focal_length_ratio  Minimum ratio of focal length over
//                                maximum sensor dimension.
// @param min_focal_length_ratio  Maximum ratio of focal length over
//                                maximum sensor dimension.
// @param max_extra_param         Maximum magnitude of each extra parameter.
bool CameraModelHasBogusParams(const int model_id,
                               const std::vector<double>& params,
                               const size_t width, const size_t height,
                               const double min_focal_length_ratio,
                               const double max_focal_length_ratio,
                               const double max_extra_param);

// Transform world coordinates in camera coordinate system to image coordinates.
//
// This is the inverse of `CameraModelImageToWorld`.
//
// @param model_id     Unique model_id of camera model as defined in
//                     `CAMERA_MODEL_NAME_TO_CODE`.
// @param params       Array of camera parameters.
// @param u, v         Coordinates in camera system as (u, v, 1).
// @param x, y         Output image coordinates in pixels.
inline void CameraModelWorldToImage(const int model_id,
                                    const std::vector<double>& params,
                                    const double u, const double v, double* x,
                                    double* y);

// Transform image coordinates to world coordinates in camera coordinate system.
//
// This is the inverse of `CameraModelWorldToImage`.
//
// @param model_id      Unique identifier of camera model.
// @param params        Array of camera parameters.
// @param x, y          Image coordinates in pixels.
// @param v, u          Output Coordinates in camera system as (u, v, 1).
inline void CameraModelImageToWorld(const int model_id,
                                    const std::vector<double>& params,
                                    const double x, const double y, double* u,
                                    double* v);

// Convert pixel threshold in image plane to world space by dividing
// the threshold through the mean focal length.   通过将阈值除以平均焦距，将像素阈值从图像平面转换到世界坐标系？？？
//
// @param model_id      Unique identifier of camera model.
// @param params        Array of camera parameters.
// @param threshold     Image space threshold in pixels.  图像平面中的阈值（以像素为单位）
//
// @ return             World space threshold.      世界坐标系中的阈值
inline double CameraModelImageToWorldThreshold(
    const int model_id, const std::vector<double>& params,
    const double threshold);

////////////////////////////////////////////////////////////////////////////////
// Implementation                                                              // 实现
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// BaseCameraModel

template <typename CameraModel>  // 这两个template声明分别为模板类和模板函数指定了类型参数
template <typename T>            // CameraModel 用于指定相机模型类型，而 T 用于指定参数的类型
bool BaseCameraModel<CameraModel>::HasBogusParams(
    const std::vector<T>& params, const size_t width, const size_t height,
    const T min_focal_length_ratio, const T max_focal_length_ratio,
    const T max_extra_param) {
  if (HasBogusPrincipalPoint(params, width, height)) {
    return true;
  }

  if (HasBogusFocalLength(params, width, height, min_focal_length_ratio,
                          max_focal_length_ratio)) {
    return true;
  }

  if (HasBogusExtraParams(params, max_extra_param)) {
    return true;
  }

  return false;
}

template <typename CameraModel>
template <typename T>
bool BaseCameraModel<CameraModel>::HasBogusFocalLength(
    const std::vector<T>& params, const size_t width, const size_t height,
    const T min_focal_length_ratio, const T max_focal_length_ratio) {
  const size_t max_size = std::max(width, height);

  for (const auto& idx : CameraModel::focal_length_idxs) {  //  CameraModel::focal_length_idxs 是一个静态成员变量，它存储了相机模型中的焦距参数的索引 是一个容器（std::vector<size_t>），包含多个焦距参数在 params 数组中的索引
    const T focal_length_ratio = params[idx] / max_size;   // 计算焦距参数与最大传感器尺寸max(width, height)的比率
    if (focal_length_ratio < min_focal_length_ratio ||
        focal_length_ratio > max_focal_length_ratio) {
      return true;   // 超出范围 返回true
    }
  }

  return false;
}

template <typename CameraModel>
template <typename T>
bool BaseCameraModel<CameraModel>::HasBogusPrincipalPoint(
    const std::vector<T>& params, const size_t width, const size_t height) {
  const T cx = params[CameraModel::principal_point_idxs[0]];  // 从 params 向量中获取主点的 x 和 y 坐标。CameraModel::principal_point_idxs 是一个静态成员变量，存储主点的索引
  const T cy = params[CameraModel::principal_point_idxs[1]];
  return cx < 0 || cx > width || cy < 0 || cy > height;  // 如果主点坐标超出图像边界，则返回 true，表示主点有误
}

template <typename CameraModel>
template <typename T>
bool BaseCameraModel<CameraModel>::HasBogusExtraParams(
    const std::vector<T>& params, const T max_extra_param) {
  for (const auto& idx : CameraModel::extra_params_idxs) {
    if (std::abs(params[idx]) > max_extra_param) {     // 怎么使用的后续  我看很多都不一样呢  用到再看吧zbw？？？
      return true;
    }
  }

  return false;
}

template <typename CameraModel>
template <typename T>
T BaseCameraModel<CameraModel>::ImageToWorldThreshold(const T* params,
                                                      const T threshold) {
  T mean_focal_length = 0;
  for (const auto& idx : CameraModel::focal_length_idxs) {
    mean_focal_length += params[idx];
  }
  mean_focal_length /= CameraModel::focal_length_idxs.size();
  return threshold / mean_focal_length;     // 也没有问题，但是这个阈值用法为？？？zbw
}

template <typename CameraModel>
template <typename T>
void BaseCameraModel<CameraModel>::IterativeUndistortion(const T* params, T* u,
                                                         T* v) {
  // Parameters for Newton iteration using numerical differentiation with    使用数值微分的中央差分法进行牛顿迭代的参数
  // central differences, 100 iterations should be enough even for complex    对于复杂的相机模型以及高阶项，100次迭代应该足够
  // camera models with higher order terms.
  const size_t kNumIterations = 100;
  const double kMaxStepNorm = 1e-10;
  const double kRelStepSize = 1e-6;

  Eigen::Matrix2d J;
  const Eigen::Vector2d x0(*u, *v);
  Eigen::Vector2d x(*u, *v);
  Eigen::Vector2d dx;
  Eigen::Vector2d dx_0b;
  Eigen::Vector2d dx_0f;
  Eigen::Vector2d dx_1b;
  Eigen::Vector2d dx_1f;

  for (size_t i = 0; i < kNumIterations; ++i) {
    const double step0 = std::max(std::numeric_limits<double>::epsilon(),
                                  std::abs(kRelStepSize * x(0)));
    const double step1 = std::max(std::numeric_limits<double>::epsilon(),
                                  std::abs(kRelStepSize * x(1)));
    CameraModel::Distortion(params, x(0), x(1), &dx(0), &dx(1));
    CameraModel::Distortion(params, x(0) - step0, x(1), &dx_0b(0), &dx_0b(1));
    CameraModel::Distortion(params, x(0) + step0, x(1), &dx_0f(0), &dx_0f(1));
    CameraModel::Distortion(params, x(0), x(1) - step1, &dx_1b(0), &dx_1b(1));
    CameraModel::Distortion(params, x(0), x(1) + step1, &dx_1f(0), &dx_1f(1));
    J(0, 0) = 1 + (dx_0f(0) - dx_0b(0)) / (2 * step0);
    J(0, 1) = (dx_1f(0) - dx_1b(0)) / (2 * step1);
    J(1, 0) = (dx_0f(1) - dx_0b(1)) / (2 * step0);
    J(1, 1) = 1 + (dx_1f(1) - dx_1b(1)) / (2 * step1);
    const Eigen::Vector2d step_x = J.inverse() * (x + dx - x0);
    x -= step_x;
    if (step_x.squaredNorm() < kMaxStepNorm) {
      break;
    }
  }

  *u = x(0);
  *v = x(1);
}

////////////////////////////////////////////////////////////////////////////////
// SimplePinholeCameraModel     3dgs说是用的是这个  但是我觉得哦我写的其实不是 好像是opencv呢

std::string SimplePinholeCameraModel::InitializeParamsInfo() {
  return "f, cx, cy";
}

std::vector<size_t> SimplePinholeCameraModel::InitializeFocalLengthIdxs() {  // 返回焦距参数的索引列表
  return {0};  // 返回一个包含一个元素 0 的向量。这个元素 0 代表了焦距参数的索引位置（索引为 0）
}

std::vector<size_t> SimplePinholeCameraModel::InitializePrincipalPointIdxs() {
  return {1, 2};
}

std::vector<size_t> SimplePinholeCameraModel::InitializeExtraParamsIdxs() {
  return {};
}

std::vector<double> SimplePinholeCameraModel::InitializeParams(
    const double focal_length, const size_t width, const size_t height) {
  return {focal_length, width / 2.0, height / 2.0};    // cx 和 cy 一般表示相机的主点坐标（principal point），其默认值通常是图像宽度和高度的一半!!!!
}

template <typename T>
void SimplePinholeCameraModel::WorldToImage(const T* params, const T u,
                                            const T v, T* x, T* y) {
  const T f = params[0];
  const T c1 = params[1];
  const T c2 = params[2];

  // No Distortion

  // Transform to image coordinates
  *x = f * u + c1;
  *y = f * v + c2;
}

template <typename T>
void SimplePinholeCameraModel::ImageToWorld(const T* params, const T x,
                                            const T y, T* u, T* v) {
  const T f = params[0];
  const T c1 = params[1];
  const T c2 = params[2];

  *u = (x - c1) / f;
  *v = (y - c2) / f;
}

////////////////////////////////////////////////////////////////////////////////
// PinholeCameraModel

std::string PinholeCameraModel::InitializeParamsInfo() {
  return "fx, fy, cx, cy";
}

std::vector<size_t> PinholeCameraModel::InitializeFocalLengthIdxs() {
  return {0, 1};
}

std::vector<size_t> PinholeCameraModel::InitializePrincipalPointIdxs() {
  return {2, 3};
}

std::vector<size_t> PinholeCameraModel::InitializeExtraParamsIdxs() {
  return {};
}

std::vector<double> PinholeCameraModel::InitializeParams(
    const double focal_length, const size_t width, const size_t height) {
  return {focal_length, focal_length, width / 2.0, height / 2.0};
}

template <typename T>
void PinholeCameraModel::WorldToImage(const T* params, const T u, const T v,
                                      T* x, T* y) {
  const T f1 = params[0];
  const T f2 = params[1];
  const T c1 = params[2];
  const T c2 = params[3];

  // No Distortion

  // Transform to image coordinates
  *x = f1 * u + c1;
  *y = f2 * v + c2;
}

template <typename T>
void PinholeCameraModel::ImageToWorld(const T* params, const T x, const T y,
                                      T* u, T* v) {
  const T f1 = params[0];
  const T f2 = params[1];
  const T c1 = params[2];
  const T c2 = params[3];

  *u = (x - c1) / f1;
  *v = (y - c2) / f2;
}

////////////////////////////////////////////////////////////////////////////////
// SimpleRadialCameraModel

std::string SimpleRadialCameraModel::InitializeParamsInfo() {
  return "f, cx, cy, k";
}

std::vector<size_t> SimpleRadialCameraModel::InitializeFocalLengthIdxs() {
  return {0};
}

std::vector<size_t> SimpleRadialCameraModel::InitializePrincipalPointIdxs() {
  return {1, 2};
}

std::vector<size_t> SimpleRadialCameraModel::InitializeExtraParamsIdxs() {
  return {3};
}

std::vector<double> SimpleRadialCameraModel::InitializeParams(
    const double focal_length, const size_t width, const size_t height) {
  return {focal_length, width / 2.0, height / 2.0, 0};
}

template <typename T>
void SimpleRadialCameraModel::WorldToImage(const T* params, const T u,
                                           const T v, T* x, T* y) {
  const T f = params[0];
  const T c1 = params[1];
  const T c2 = params[2];

  // Distortion
  T du, dv;
  Distortion(&params[3], u, v, &du, &dv);
  *x = u + du;
  *y = v + dv;

  // Transform to image coordinates
  *x = f * *x + c1;
  *y = f * *y + c2;
}

template <typename T>
void SimpleRadialCameraModel::ImageToWorld(const T* params, const T x,
                                           const T y, T* u, T* v) {
  const T f = params[0];
  const T c1 = params[1];
  const T c2 = params[2];

  // Lift points to normalized plane
  *u = (x - c1) / f;
  *v = (y - c2) / f;

  IterativeUndistortion(&params[3], u, v);
}

template <typename T>
void SimpleRadialCameraModel::Distortion(const T* extra_params, const T u,
                                         const T v, T* du, T* dv) {
  const T k = extra_params[0];

  const T u2 = u * u;
  const T v2 = v * v;
  const T r2 = u2 + v2;
  const T radial = k * r2;
  *du = u * radial;
  *dv = v * radial;
}

////////////////////////////////////////////////////////////////////////////////
// RadialCameraModel

std::string RadialCameraModel::InitializeParamsInfo() {
  return "f, cx, cy, k1, k2";
}

std::vector<size_t> RadialCameraModel::InitializeFocalLengthIdxs() {
  return {0};
}

std::vector<size_t> RadialCameraModel::InitializePrincipalPointIdxs() {
  return {1, 2};
}

std::vector<size_t> RadialCameraModel::InitializeExtraParamsIdxs() {
  return {3, 4};
}

std::vector<double> RadialCameraModel::InitializeParams(
    const double focal_length, const size_t width, const size_t height) {
  return {focal_length, width / 2.0, height / 2.0, 0, 0};
}

template <typename T>
void RadialCameraModel::WorldToImage(const T* params, const T u, const T v,
                                     T* x, T* y) {
  const T f = params[0];
  const T c1 = params[1];
  const T c2 = params[2];

  // Distortion
  T du, dv;
  Distortion(&params[3], u, v, &du, &dv);  // 调用 Distortion 函数来计算 (u, v) 点的畸变影响
  *x = u + du;
  *y = v + dv;  // 将畸变调整后的坐标 (u + du, v + dv) 存储到输出变量 x 和 y 中

  // Transform to image coordinates
  *x = f * *x + c1;  // 将畸变调整后的横坐标 *x 乘以焦距 f，并加上主点的横坐标 c1，得到图像坐标系中的横坐标   这个人不加括号Orz！
  *y = f * *y + c2;
}

template <typename T>
void RadialCameraModel::ImageToWorld(const T* params, const T x, const T y,
                                     T* u, T* v) {
  const T f = params[0];
  const T c1 = params[1];
  const T c2 = params[2];

  // Lift points to normalized plane
  *u = (x - c1) / f;
  *v = (y - c2) / f;

  IterativeUndistortion(&params[3], u, v);
}

template <typename T>
void RadialCameraModel::Distortion(const T* extra_params, const T u, const T v,
                                   T* du, T* dv) {
  const T k1 = extra_params[0];
  const T k2 = extra_params[1];

  const T u2 = u * u;
  const T v2 = v * v;
  const T r2 = u2 + v2;
  const T radial = k1 * r2 + k2 * r2 * r2;
  *du = u * radial;
  *dv = v * radial;
}

////////////////////////////////////////////////////////////////////////////////
// OpenCVCameraModel

std::string OpenCVCameraModel::InitializeParamsInfo() {
  return "fx, fy, cx, cy, k1, k2, p1, p2";
}

std::vector<size_t> OpenCVCameraModel::InitializeFocalLengthIdxs() {
  return {0, 1};
}

std::vector<size_t> OpenCVCameraModel::InitializePrincipalPointIdxs() {
  return {2, 3};
}

std::vector<size_t> OpenCVCameraModel::InitializeExtraParamsIdxs() {
  return {4, 5, 6, 7};
}

std::vector<double> OpenCVCameraModel::InitializeParams(
    const double focal_length, const size_t width, const size_t height) {
  return {focal_length, focal_length, width / 2.0, height / 2.0, 0, 0, 0, 0};
}

template <typename T>
void OpenCVCameraModel::WorldToImage(const T* params, const T u, const T v,
                                     T* x, T* y) {
  const T f1 = params[0];
  const T f2 = params[1];
  const T c1 = params[2];
  const T c2 = params[3];

  // Distortion
  T du, dv;
  Distortion(&params[4], u, v, &du, &dv);
  *x = u + du;
  *y = v + dv;

  // Transform to image coordinates
  *x = f1 * *x + c1;
  *y = f2 * *y + c2;
}

template <typename T>
void OpenCVCameraModel::ImageToWorld(const T* params, const T x, const T y,
                                     T* u, T* v) {
  const T f1 = params[0];
  const T f2 = params[1];
  const T c1 = params[2];
  const T c2 = params[3];

  // Lift points to normalized plane
  *u = (x - c1) / f1;
  *v = (y - c2) / f2;

  IterativeUndistortion(&params[4], u, v);
}

template <typename T>
void OpenCVCameraModel::Distortion(const T* extra_params, const T u, const T v,
                                   T* du, T* dv) {
  const T k1 = extra_params[0];
  const T k2 = extra_params[1];
  const T p1 = extra_params[2];
  const T p2 = extra_params[3];

  const T u2 = u * u;
  const T uv = u * v;
  const T v2 = v * v;
  const T r2 = u2 + v2;
  const T radial = k1 * r2 + k2 * r2 * r2;
  *du = u * radial + T(2) * p1 * uv + p2 * (r2 + T(2) * u2);
  *dv = v * radial + T(2) * p2 * uv + p1 * (r2 + T(2) * v2);
}

////////////////////////////////////////////////////////////////////////////////
// OpenCVFisheyeCameraModel

std::string OpenCVFisheyeCameraModel::InitializeParamsInfo() {
  return "fx, fy, cx, cy, k1, k2, k3, k4";
}

std::vector<size_t> OpenCVFisheyeCameraModel::InitializeFocalLengthIdxs() {
  return {0, 1};
}

std::vector<size_t> OpenCVFisheyeCameraModel::InitializePrincipalPointIdxs() {
  return {2, 3};
}

std::vector<size_t> OpenCVFisheyeCameraModel::InitializeExtraParamsIdxs() {
  return {4, 5, 6, 7};
}

std::vector<double> OpenCVFisheyeCameraModel::InitializeParams(
    const double focal_length, const size_t width, const size_t height) {
  return {focal_length, focal_length, width / 2.0, height / 2.0, 0, 0, 0, 0};
}

template <typename T>
void OpenCVFisheyeCameraModel::WorldToImage(const T* params, const T u,
                                            const T v, T* x, T* y) {
  const T f1 = params[0];
  const T f2 = params[1];
  const T c1 = params[2];
  const T c2 = params[3];

  // Distortion
  T du, dv;
  Distortion(&params[4], u, v, &du, &dv);
  *x = u + du;
  *y = v + dv;

  // Transform to image coordinates
  *x = f1 * *x + c1;
  *y = f2 * *y + c2;
}

template <typename T>
void OpenCVFisheyeCameraModel::ImageToWorld(const T* params, const T x,
                                            const T y, T* u, T* v) {
  const T f1 = params[0];
  const T f2 = params[1];
  const T c1 = params[2];
  const T c2 = params[3];

  // Lift points to normalized plane
  *u = (x - c1) / f1;
  *v = (y - c2) / f2;

  IterativeUndistortion(&params[4], u, v);
}

template <typename T>
void OpenCVFisheyeCameraModel::Distortion(const T* extra_params, const T u,
                                          const T v, T* du, T* dv) {
  const T k1 = extra_params[0];
  const T k2 = extra_params[1];
  const T k3 = extra_params[2];
  const T k4 = extra_params[3];

  const T r = ceres::sqrt(u * u + v * v);

  if (r > T(std::numeric_limits<double>::epsilon())) {
    const T theta = ceres::atan(r);
    const T theta2 = theta * theta;
    const T theta4 = theta2 * theta2;
    const T theta6 = theta4 * theta2;
    const T theta8 = theta4 * theta4;
    const T thetad =
        theta * (T(1) + k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8);
    *du = u * thetad / r - u;
    *dv = v * thetad / r - v;
  } else {
    *du = T(0);
    *dv = T(0);
  }
}

////////////////////////////////////////////////////////////////////////////////
// FullOpenCVCameraModel

std::string FullOpenCVCameraModel::InitializeParamsInfo() {
  return "fx, fy, cx, cy, k1, k2, p1, p2, k3, k4, k5, k6";
}

std::vector<size_t> FullOpenCVCameraModel::InitializeFocalLengthIdxs() {
  return {0, 1};
}

std::vector<size_t> FullOpenCVCameraModel::InitializePrincipalPointIdxs() {
  return {2, 3};
}

std::vector<size_t> FullOpenCVCameraModel::InitializeExtraParamsIdxs() {
  return {4, 5, 6, 7, 8, 9, 10, 11};
}

std::vector<double> FullOpenCVCameraModel::InitializeParams(
    const double focal_length, const size_t width, const size_t height) {
  return {focal_length,
          focal_length,
          width / 2.0,
          height / 2.0,
          0,
          0,
          0,
          0,
          0,
          0,
          0,
          0};
}

template <typename T>
void FullOpenCVCameraModel::WorldToImage(const T* params, const T u, const T v,
                                         T* x, T* y) {
  const T f1 = params[0];
  const T f2 = params[1];
  const T c1 = params[2];
  const T c2 = params[3];

  // Distortion
  T du, dv;
  Distortion(&params[4], u, v, &du, &dv);
  *x = u + du;
  *y = v + dv;

  // Transform to image coordinates
  *x = f1 * *x + c1;
  *y = f2 * *y + c2;
}

template <typename T>
void FullOpenCVCameraModel::ImageToWorld(const T* params, const T x, const T y,
                                         T* u, T* v) {
  const T f1 = params[0];
  const T f2 = params[1];
  const T c1 = params[2];
  const T c2 = params[3];

  // Lift points to normalized plane
  *u = (x - c1) / f1;
  *v = (y - c2) / f2;

  IterativeUndistortion(&params[4], u, v);
}

template <typename T>
void FullOpenCVCameraModel::Distortion(const T* extra_params, const T u,
                                       const T v, T* du, T* dv) {
  const T k1 = extra_params[0];
  const T k2 = extra_params[1];
  const T p1 = extra_params[2];
  const T p2 = extra_params[3];
  const T k3 = extra_params[4];
  const T k4 = extra_params[5];
  const T k5 = extra_params[6];
  const T k6 = extra_params[7];

  const T u2 = u * u;
  const T uv = u * v;
  const T v2 = v * v;
  const T r2 = u2 + v2;
  const T r4 = r2 * r2;
  const T r6 = r4 * r2;
  const T radial = (T(1) + k1 * r2 + k2 * r4 + k3 * r6) /
                   (T(1) + k4 * r2 + k5 * r4 + k6 * r6);
  *du = u * radial + T(2) * p1 * uv + p2 * (r2 + T(2) * u2) - u;
  *dv = v * radial + T(2) * p2 * uv + p1 * (r2 + T(2) * v2) - v;
}

////////////////////////////////////////////////////////////////////////////////
// FOVCameraModel

std::string FOVCameraModel::InitializeParamsInfo() {
  return "fx, fy, cx, cy, omega";
}

std::vector<size_t> FOVCameraModel::InitializeFocalLengthIdxs() {
  return {0, 1};
}

std::vector<size_t> FOVCameraModel::InitializePrincipalPointIdxs() {
  return {2, 3};
}

std::vector<size_t> FOVCameraModel::InitializeExtraParamsIdxs() { return {4}; }

std::vector<double> FOVCameraModel::InitializeParams(const double focal_length,
                                                     const size_t width,
                                                     const size_t height) {
  return {focal_length, focal_length, width / 2.0, height / 2.0, 1e-2};
}

template <typename T>
void FOVCameraModel::WorldToImage(const T* params, const T u, const T v, T* x,
                                  T* y) {
  const T f1 = params[0];
  const T f2 = params[1];
  const T c1 = params[2];
  const T c2 = params[3];

  // Distortion
  Distortion(&params[4], u, v, x, y);

  // Transform to image coordinates
  *x = f1 * *x + c1;
  *y = f2 * *y + c2;
}

template <typename T>
void FOVCameraModel::ImageToWorld(const T* params, const T x, const T y, T* u,
                                  T* v) {
  const T f1 = params[0];
  const T f2 = params[1];
  const T c1 = params[2];
  const T c2 = params[3];

  // Lift points to normalized plane
  const T uu = (x - c1) / f1;
  const T vv = (y - c2) / f2;

  // Undistortion
  Undistortion(&params[4], uu, vv, u, v);
}

template <typename T>
void FOVCameraModel::Distortion(const T* extra_params, const T u, const T v,
                                T* du, T* dv) {
  const T omega = extra_params[0];

  // Chosen arbitrarily.
  const T kEpsilon = T(1e-4);

  const T radius2 = u * u + v * v;
  const T omega2 = omega * omega;

  T factor;
  if (omega2 < kEpsilon) {
    // Derivation of this case with Matlab:
    // syms radius omega;
    // factor(radius) = atan(radius * 2 * tan(omega / 2)) / ...
    //                  (radius * omega);
    // simplify(taylor(factor, omega, 'order', 3))
    factor = (omega2 * radius2) / T(3) - omega2 / T(12) + T(1);
  } else if (radius2 < kEpsilon) {
    // Derivation of this case with Matlab:
    // syms radius omega;
    // factor(radius) = atan(radius * 2 * tan(omega / 2)) / ...
    //                  (radius * omega);
    // simplify(taylor(factor, radius, 'order', 3))
    const T tan_half_omega = ceres::tan(omega / T(2));
    factor = (T(-2) * tan_half_omega *
              (T(4) * radius2 * tan_half_omega * tan_half_omega - T(3))) /
             (T(3) * omega);
  } else {
    const T radius = ceres::sqrt(radius2);
    const T numerator = ceres::atan(radius * T(2) * ceres::tan(omega / T(2)));
    factor = numerator / (radius * omega);
  }

  *du = u * factor;
  *dv = v * factor;
}

template <typename T>
void FOVCameraModel::Undistortion(const T* extra_params, const T u, const T v,
                                  T* du, T* dv) {
  T omega = extra_params[0];

  // Chosen arbitrarily.
  const T kEpsilon = T(1e-4);

  const T radius2 = u * u + v * v;
  const T omega2 = omega * omega;

  T factor;
  if (omega2 < kEpsilon) {
    // Derivation of this case with Matlab:
    // syms radius omega;
    // factor(radius) = tan(radius * omega) / ...
    //                  (radius * 2*tan(omega/2));
    // simplify(taylor(factor, omega, 'order', 3))
    factor = (omega2 * radius2) / T(3) - omega2 / T(12) + T(1);
  } else if (radius2 < kEpsilon) {
    // Derivation of this case with Matlab:
    // syms radius omega;
    // factor(radius) = tan(radius * omega) / ...
    //                  (radius * 2*tan(omega/2));
    // simplify(taylor(factor, radius, 'order', 3))
    factor = (omega * (omega * omega * radius2 + T(3))) /
             (T(6) * ceres::tan(omega / T(2)));
  } else {
    const T radius = ceres::sqrt(radius2);
    const T numerator = ceres::tan(radius * omega);
    factor = numerator / (radius * T(2) * ceres::tan(omega / T(2)));
  }

  *du = u * factor;
  *dv = v * factor;
}

////////////////////////////////////////////////////////////////////////////////
// SimpleRadialFisheyeCameraModel

std::string SimpleRadialFisheyeCameraModel::InitializeParamsInfo() {
  return "f, cx, cy, k";
}

std::vector<size_t>
SimpleRadialFisheyeCameraModel::InitializeFocalLengthIdxs() {
  return {0};
}

std::vector<size_t>
SimpleRadialFisheyeCameraModel::InitializePrincipalPointIdxs() {
  return {1, 2};
}

std::vector<size_t>
SimpleRadialFisheyeCameraModel::InitializeExtraParamsIdxs() {
  return {3};
}

std::vector<double> SimpleRadialFisheyeCameraModel::InitializeParams(
    const double focal_length, const size_t width, const size_t height) {
  return {focal_length, width / 2.0, height / 2.0, 0};
}

template <typename T>
void SimpleRadialFisheyeCameraModel::WorldToImage(const T* params, const T u,
                                                  const T v, T* x, T* y) {
  const T f = params[0];
  const T c1 = params[1];
  const T c2 = params[2];

  // Distortion
  T du, dv;
  Distortion(&params[3], u, v, &du, &dv);
  *x = u + du;
  *y = v + dv;

  // Transform to image coordinates
  *x = f * *x + c1;
  *y = f * *y + c2;
}

template <typename T>
void SimpleRadialFisheyeCameraModel::ImageToWorld(const T* params, const T x,
                                                  const T y, T* u, T* v) {
  const T f = params[0];
  const T c1 = params[1];
  const T c2 = params[2];

  // Lift points to normalized plane
  *u = (x - c1) / f;
  *v = (y - c2) / f;

  IterativeUndistortion(&params[3], u, v);
}

template <typename T>
void SimpleRadialFisheyeCameraModel::Distortion(const T* extra_params,
                                                const T u, const T v, T* du,
                                                T* dv) {
  const T k = extra_params[0];

  const T r = ceres::sqrt(u * u + v * v);

  if (r > T(std::numeric_limits<double>::epsilon())) {
    const T theta = ceres::atan(r);
    const T theta2 = theta * theta;
    const T thetad = theta * (T(1) + k * theta2);
    *du = u * thetad / r - u;
    *dv = v * thetad / r - v;
  } else {
    *du = T(0);
    *dv = T(0);
  }
}

////////////////////////////////////////////////////////////////////////////////
// RadialFisheyeCameraModel

std::string RadialFisheyeCameraModel::InitializeParamsInfo() {
  return "f, cx, cy, k1, k2";
}

std::vector<size_t> RadialFisheyeCameraModel::InitializeFocalLengthIdxs() {
  return {0};
}

std::vector<size_t> RadialFisheyeCameraModel::InitializePrincipalPointIdxs() {
  return {1, 2};
}

std::vector<size_t> RadialFisheyeCameraModel::InitializeExtraParamsIdxs() {
  return {3, 4};
}

std::vector<double> RadialFisheyeCameraModel::InitializeParams(
    const double focal_length, const size_t width, const size_t height) {
  return {focal_length, width / 2.0, height / 2.0, 0, 0};
}

template <typename T>
void RadialFisheyeCameraModel::WorldToImage(const T* params, const T u,
                                            const T v, T* x, T* y) {
  const T f = params[0];
  const T c1 = params[1];
  const T c2 = params[2];

  // Distortion
  T du, dv;
  Distortion(&params[3], u, v, &du, &dv);
  *x = u + du;
  *y = v + dv;

  // Transform to image coordinates
  *x = f * *x + c1;
  *y = f * *y + c2;
}

template <typename T>
void RadialFisheyeCameraModel::ImageToWorld(const T* params, const T x,
                                            const T y, T* u, T* v) {
  const T f = params[0];
  const T c1 = params[1];
  const T c2 = params[2];

  // Lift points to normalized plane
  *u = (x - c1) / f;
  *v = (y - c2) / f;

  IterativeUndistortion(&params[3], u, v);
}

template <typename T>
void RadialFisheyeCameraModel::Distortion(const T* extra_params, const T u,
                                          const T v, T* du, T* dv) {
  const T k1 = extra_params[0];
  const T k2 = extra_params[1];

  const T r = ceres::sqrt(u * u + v * v);

  if (r > T(std::numeric_limits<double>::epsilon())) {
    const T theta = ceres::atan(r);
    const T theta2 = theta * theta;
    const T theta4 = theta2 * theta2;
    const T thetad =
        theta * (T(1) + k1 * theta2 + k2 * theta4);
    *du = u * thetad / r - u;
    *dv = v * thetad / r - v;
  } else {
    *du = T(0);
    *dv = T(0);
  }
}

////////////////////////////////////////////////////////////////////////////////
// ThinPrismFisheyeCameraModel

std::string ThinPrismFisheyeCameraModel::InitializeParamsInfo() {
  return "fx, fy, cx, cy, k1, k2, p1, p2, k3, k4, sx1, sy1";
}

std::vector<size_t> ThinPrismFisheyeCameraModel::InitializeFocalLengthIdxs() {
  return {0, 1};
}

std::vector<size_t>
ThinPrismFisheyeCameraModel::InitializePrincipalPointIdxs() {
  return {2, 3};
}

std::vector<size_t> ThinPrismFisheyeCameraModel::InitializeExtraParamsIdxs() {
  return {4, 5, 6, 7, 8, 9, 10, 11};
}

std::vector<double> ThinPrismFisheyeCameraModel::InitializeParams(
    const double focal_length, const size_t width, const size_t height) {
  return {focal_length,
          focal_length,
          width / 2.0,
          height / 2.0,
          0,
          0,
          0,
          0,
          0,
          0,
          0,
          0};
}

template <typename T>
void ThinPrismFisheyeCameraModel::WorldToImage(const T* params, const T u,
                                               const T v, T* x, T* y) {
  const T f1 = params[0];
  const T f2 = params[1];
  const T c1 = params[2];
  const T c2 = params[3];

  const T r = ceres::sqrt(u * u + v * v);

  T uu, vv;
  if (r > T(std::numeric_limits<double>::epsilon())) {
    const T theta = ceres::atan(r);
    uu = theta * u / r;
    vv = theta * v / r;
  } else {
    uu = u;
    vv = v;
  }

  // Distortion
  T du, dv;
  Distortion(&params[4], uu, vv, &du, &dv);
  *x = uu + du;
  *y = vv + dv;

  // Transform to image coordinates
  *x = f1 * *x + c1;
  *y = f2 * *y + c2;
}

template <typename T>
void ThinPrismFisheyeCameraModel::ImageToWorld(const T* params, const T x,
                                               const T y, T* u, T* v) {
  const T f1 = params[0];
  const T f2 = params[1];
  const T c1 = params[2];
  const T c2 = params[3];

  // Lift points to normalized plane
  *u = (x - c1) / f1;
  *v = (y - c2) / f2;

  IterativeUndistortion(&params[4], u, v);

  const T theta = ceres::sqrt(*u * *u + *v * *v);
  const T theta_cos_theta = theta * ceres::cos(theta);
  if (theta_cos_theta > T(std::numeric_limits<double>::epsilon())) {
    const T scale = ceres::sin(theta) / theta_cos_theta;
    *u *= scale;
    *v *= scale;
  }
}

template <typename T>
void ThinPrismFisheyeCameraModel::Distortion(const T* extra_params, const T u,
                                             const T v, T* du, T* dv) {
  const T k1 = extra_params[0];
  const T k2 = extra_params[1];
  const T p1 = extra_params[2];
  const T p2 = extra_params[3];
  const T k3 = extra_params[4];
  const T k4 = extra_params[5];
  const T sx1 = extra_params[6];
  const T sy1 = extra_params[7];

  const T u2 = u * u;
  const T uv = u * v;
  const T v2 = v * v;
  const T r2 = u2 + v2;
  const T r4 = r2 * r2;
  const T r6 = r4 * r2;
  const T r8 = r6 * r2;
  const T radial = k1 * r2 + k2 * r4 + k3 * r6 + k4 * r8;
  *du = u * radial + T(2) * p1 * uv + p2 * (r2 + T(2) * u2) + sx1 * r2;
  *dv = v * radial + T(2) * p2 * uv + p1 * (r2 + T(2) * v2) + sy1 * r2;
}

////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
void CameraModelWorldToImage(const int model_id,                               // 根据不同的相机模型id选择对应的方法
                             const std::vector<double>& params, const double u,
                             const double v, double* x, double* y) {
  switch (model_id) {
#define CAMERA_MODEL_CASE(CameraModel)                    \
  case CameraModel::kModelId:                             \
    CameraModel::WorldToImage(params.data(), u, v, x, y); \
    break;

    CAMERA_MODEL_SWITCH_CASES

#undef CAMERA_MODEL_CASE
  }
}

void CameraModelImageToWorld(const int model_id,
                             const std::vector<double>& params, const double x,
                             const double y, double* u, double* v) {
  switch (model_id) {
#define CAMERA_MODEL_CASE(CameraModel)                    \
  case CameraModel::kModelId:                             \
    CameraModel::ImageToWorld(params.data(), x, y, u, v); \
    break;

    CAMERA_MODEL_SWITCH_CASES

#undef CAMERA_MODEL_CASE
  }
}

double CameraModelImageToWorldThreshold(const int model_id,
                                        const std::vector<double>& params,
                                        const double threshold) {          // 这里不对，因为阈值计算方法就一个，作者自己没写
  switch (model_id) {
#define CAMERA_MODEL_CASE(CameraModel)                                   \
  case CameraModel::kModelId:                                            \
    return CameraModel::ImageToWorldThreshold(params.data(), threshold); \
    break;

    CAMERA_MODEL_SWITCH_CASES

#undef CAMERA_MODEL_CASE
  }

  return -1;    // 这个return是CameraModelImageToWorldThreshold的返回值。如果 switch 语句中没有匹配到任何 case，程序会直接跳过 switch 块，继续执行后续的代码。因此，return -1; 是为了处理这种情况，确保函数在所有情况下都有返回值。
}

}  // namespace colmap

#endif  // COLMAP_SRC_BASE_CAMERA_MODELS_H_
