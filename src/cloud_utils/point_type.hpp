#pragma once

#define PCL_NO_PRECOMPILE

#include <pcl/point_types.h>
#include <pcl/register_point_struct.h>

struct EIGEN_ALIGN16 PointXYZIT {
  PCL_ADD_POINT4D;
  float intensity;
  uint8_t tag;
  uint8_t line;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIT, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                                                  uint8_t, tag, tag)(uint8_t, line, line)(double, timestamp, timestamp))