#include <cmath>

#pragma once
namespace float_check {
inline bool isnan(double value) { return (value != value); }

inline bool isinf(double value) { return (value > 1e300 || value < -1e300); }

inline bool isfinite(double value) { return !isnan(value) && isinf(value); }

inline bool isnan(float value) { return (value != value); }

inline bool isinf(float value) { return (value > 1e38f || value < -1e38f); }

inline bool isfinite(float value) { return !isnan(value) && !isinf(value); }
}  // namespace float_check