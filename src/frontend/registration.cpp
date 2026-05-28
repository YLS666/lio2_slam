#include "frontend/registration.hpp"
#include <tbb/tbb.h>
#include <iostream>
#include "utils/eigen_types.hpp"

Registration::Registration() {}

Eigen::Matrix3d Registration::skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d m;
  m << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;

  return m;
}

bool Registration::align(const pcl::PointCloud<PointType>::Ptr& cloud, VoxelMap* map, State& state) {
  if (map->size() < 100) {
    return false;
  }

  constexpr int MAX_ITER = 3;
  constexpr float MAX_DIST = 1.0f;

  for (int iter = 0; iter < MAX_ITER; ++iter) {
    const Eigen::Matrix3d R = state.q.toRotationMatrix();
    const Eigen::Vector3d t = state.p;

    struct Reduction {
      Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
      Eigen::Matrix<double, 6, 1> b = Eigen::Matrix<double, 6, 1>::Zero();
      double error = 0.0;
      int count = 0;
    };

    Reduction result = tbb::parallel_reduce(
        tbb::blocked_range<size_t>(0, cloud->size(), 512), Reduction(),
        [&](const tbb::blocked_range<size_t>& range, Reduction local) -> Reduction {
          Eigen::Matrix<double, 3, 6> J;
          Eigen::Matrix3d skew_p;

          for (size_t i = range.begin(); i < range.end(); ++i) {
            const auto& pt = cloud->points[i];

            Eigen::Vector3d p(pt.x, pt.y, pt.z);
            Eigen::Vector3d pw = R * p + t;

            PointType search_pt;
            search_pt.x = pw.x();
            search_pt.y = pw.y();
            search_pt.z = pw.z();

            PointType nearest_pt;
            float dist;

            if (!map->nearestSearch(search_pt, nearest_pt, dist)) {
              continue;
            }

            if (dist > MAX_DIST) {
              continue;
            }

            Eigen::Vector3d q(nearest_pt.x, nearest_pt.y, nearest_pt.z);
            Eigen::Vector3d residual = pw - q;

            local.error += residual.squaredNorm();

            skew_p << 0.0, -p.z(), p.y(), p.z(), 0.0, -p.x(), -p.y(), p.x(), 0.0;

            J.setZero();
            J.block<3, 3>(0, 0).setIdentity();
            J.block<3, 3>(0, 3).noalias() = -R * skew_p;

            local.H.noalias() += J.transpose() * J;
            local.b.noalias() += -J.transpose() * residual;
            local.count++;
          }

          return local;
        },

        [](const Reduction& a, const Reduction& b) {
          Reduction out;

          out.H = a.H + b.H;
          out.b = a.b + b.b;
          out.error = a.error + b.error;
          out.count = a.count + b.count;
          return out;
        });

    if (result.count < 50) {
      return false;
    }

    Eigen::Matrix<double, 6, 1> dx = result.H.ldlt().solve(result.b);

    state.p += dx.head<3>();

    Eigen::Vector3d dtheta = dx.tail<3>();
    Eigen::Quaterniond dq(1.0, 0.5 * dtheta.x(), 0.5 * dtheta.y(), 0.5 * dtheta.z());
    dq.normalize();

    state.q = state.q * dq;
    state.q.normalize();

    std::cout << "iter = " << iter << " error = " << result.error / result.count << std::endl;

    if (dx.norm() < 1e-4) {
      break;
    }
  }

  return true;
}