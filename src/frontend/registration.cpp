#include "frontend/registration.hpp"
#include <Eigen/src/Geometry/Quaternion.h>
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

  // 预计算旋转矩阵
  for (int iter = 0; iter < MAX_ITER; ++iter) {
    const Eigen::Matrix3d R = state.q.toRotationMatrix();
    const Eigen::Vector3d t = state.p;

    // 使用固定尺寸矩阵
    struct Reduction {
      Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
      Eigen::Matrix<double, 6, 1> b = Eigen::Matrix<double, 6, 1>::Zero();
      double error = 0.0;
      int count = 0;
    };

    Reduction result = tbb::parallel_reduce(
        tbb::blocked_range<size_t>(0, cloud->size(), 512), Reduction(),
        [&](const tbb::blocked_range<size_t>& range, Reduction local) -> Reduction {
          // 将 Jacobian矩阵放在循环外，防止反复构造
          // J = [I, -R * skew(p)] 是 3x6 矩阵
          Eigen::Matrix<double, 3, 6> J;
          Eigen::Matrix3d skew_p;
          Eigen::Matrix<double, 3, 3> R_skew;  // 存 R * skew(p) 结果

          for (size_t i = range.begin(); i < range.end(); ++i) {
            const auto& pt = cloud->points[i];

            const double px = pt.x;
            const double py = pt.y;
            const double pz = pt.z;

            // 不使用Eigen::Vector3d pw = R * p + t， 减少模板开销
            const double pw_x = R(0, 0) * px + R(0, 1) * py + R(0, 2) * pz + t.x();
            const double pw_y = R(1, 0) * px + R(1, 1) * py + R(1, 2) * pz + t.y();
            const double pw_z = R(2, 0) * px + R(2, 1) * py + R(2, 2) * pz + t.z();

            PointType search_pt;
            search_pt.x = pw_x;
            search_pt.y = pw_y;
            search_pt.z = pw_z;

            PointType nearest_pt;
            float dist;

            if (!map->nearestSearch(search_pt, nearest_pt, dist)) {
              continue;
            }

            if (dist > MAX_DIST) {
              continue;
            }

            // 残差 = pw - q
            Eigen::Vector3d residual(pw_x - nearest_pt.x, pw_y - nearest_pt.y, pw_z - nearest_pt.z);

            local.error += residual.squaredNorm();

            // 构造 J = [I, -R * skew(p)]
            Eigen::Vector3d p(pt.x, pt.y, pt.z);
            J.block<3, 3>(0, 0).setIdentity();
            J.block<3, 3>(0, 3).noalias() = -R * skew(p);

            // 固定尺寸矩阵的 J^T*J 和 J^T*r 已经很快
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

    // 求解 H * dx = b ， LDLT分解
    Eigen::Matrix<double, 6, 1> dx = result.H.ldlt().solve(result.b);

    // 平移更新
    state.p += dx.head<3>();

    // 旋转更新（SO3 指数映射）
    Eigen::Vector3d dtheta = dx.tail<3>();
    double dthata_norm = dtheta.norm();

    // 使用 SO3指数映射更新旋转，比四元数近似更精确
    if (dthata_norm > 1e-10) {
      Eigen::Vector3d axis = dtheta / dthata_norm;
      double half_angle = 0.5 * dtheta.norm();
      Eigen::Quaterniond dq(std::cos(half_angle), axis.x() * std::sin(half_angle), axis.y() * std::sin(half_angle),
                            axis.z() * std::sin(half_angle));
      state.q = dq * state.q;
    } else {
      Eigen::Quaterniond dq(1.0, 0.5 * dtheta.x(), 0.5 * dtheta.y(), 0.5 * dtheta.z());
      dq.normalize();

      state.q = dq * state.q;
    }

    state.q.normalize();

    std::cout << "iter = " << iter << " error = " << result.error / result.count << std::endl;

    if (dx.norm() < 1e-4) {
      break;
    }
  }

  return true;
}