#pragma once

#include <pcl/impl/point_types.hpp>
#define PCL_NO_PRECOMPILE

#include <pcl/point_types.h>
#include <pcl/register_point_struct.h>

struct FullPointType {
  PCL_ADD_POINT4D;
  float intensity = 0;
  double timestamp = 0;

  inline FullPointType() {}
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(FullPointType,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                                                          intensity)(double, timestamp, timestamp))

using PointType = pcl::PointXYZI;