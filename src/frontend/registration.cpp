#include "frontend/registration.hpp"
#include <glog/logging.h>
#include <tbb/tbb.h>
#include <cstddef>
#include <iostream>
#include "cloud_utils/point_type.hpp"

Registration::Registration() { covariance_.setZero(); }

M3d Registration::skew(const V3d& v) {
  M3d m;
  m << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;

  return m;
}

bool Registration::align(const CloudPtr& cloud, VoxelMap* map, State& state) {
  if (map->size() < 100) {
    return false;
  }

  constexpr int MAX_ITER = 3;
  constexpr float MAX_DIST = 0.5f;  // 最大匹配距离

  inlier_count_ = 0;
  match_count_ = 0;

  // 预计算旋转矩阵
  for (int iter = 0; iter < MAX_ITER; ++iter) {
    const M3d R = state.q.toRotationMatrix();
    const V3d t = state.p;

    // 使用固定尺寸矩阵
    struct Reduction {
      Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
      Eigen::Matrix<double, 6, 1> b = Eigen::Matrix<double, 6, 1>::Zero();
      double error = 0.0;
      double error_raw = 0.0;  // 未加权的残差平方和(用于协方差)
      int count = 0;

      // 用于协方差计算的原始 Hessian (不加权)
      Eigen::Matrix<double, 6, 6> H_raw = Eigen::Matrix<double, 6, 6>::Zero();
    };

    Reduction result = tbb::parallel_reduce(
        tbb::blocked_range<size_t>(0, cloud->size(), 512), Reduction(),
        [&](const tbb::blocked_range<size_t>& range, Reduction local) -> Reduction {
          // 将 Jacobian矩阵放在循环外，防止反复构造
          // J = [I, -R * skew(p)] 是 3x6 矩阵
          Eigen::Matrix<double, 3, 6> J;
          M3d skew_p;
          V3d p_lidar;
          V3d pw;
          V3d residual_vec;
          PointType nearest_pt;
          PointType search_pt;
          float dist;

          for (size_t i = range.begin(); i < range.end(); ++i) {
            const auto& pt = cloud->points[i];

            // 1. 计算世界坐标系下的点
            p_lidar << pt.x, pt.y, pt.z;

            // 2. 转到世界坐标系
            pw = R * p_lidar + t;

            // 3. 搜索最近点
            search_pt.x = static_cast<float>(pw.x());
            search_pt.y = static_cast<float>(pw.y());
            search_pt.z = static_cast<float>(pw.z());

            if (!map->nearestSearch(search_pt, nearest_pt, dist)) {
              continue;
            }

            if (dist > MAX_DIST) {
              continue;
            }

            // 4. 残差向量
            residual_vec = pw - V3d(nearest_pt.x, nearest_pt.y, nearest_pt.z);

            // 5. Huber鲁棒核
            // 原理: 对每个匹配点计算残差范数 |r| ,如果 |r| > k (默认 0.3m), 权重 < 1.0
            // 作用: 坑洼地面/动态物体会产生大残差 → 权重降低,不会污染优化结果
            double residual_norm = residual_vec.norm();
            double weight = 1.0;

            if (use_huber_) {
              weight = huberWeight(residual_norm, huber_k_);
              if (weight < 0.01) {
                continue;
              }
            }

            // 6. 统计
            local.count++;
            local.error += weight * residual_vec.squaredNorm();
            local.error_raw += residual_vec.squaredNorm();  // 未加权误差

            // 7. 构造 J = [I, -R * skew(p_lidar)]
            J.block<3, 3>(0, 0).setIdentity();
            J.block<3, 3>(0, 3) = -R * skew(p_lidar);

            // 8. Huber加权
            double sqrt_w = std::sqrt(weight);
            Eigen::Matrix<double, 3, 6> J_w = J * sqrt_w;
            V3d r_w = residual_vec * sqrt_w;

            local.H.noalias() += J_w.transpose() * J_w;
            local.b.noalias() += -J_w.transpose() * r_w;

            // 9. 不加权
            local.H_raw.noalias() += J.transpose() * J;
          }

          return local;
        },

        [](const Reduction& a, const Reduction& b) {
          Reduction out;

          out.H = a.H + b.H;
          out.b = a.b + b.b;
          out.error = a.error + b.error;
          out.error_raw = a.error_raw + b.error_raw;
          out.count = a.count + b.count;
          out.H_raw = a.H_raw + b.H_raw;
          return out;
        });

    if (result.count < 50) {
      LOG(WARNING) << "配准失败：匹配点数不足（" << result.count << ")";
      return false;
    }

    match_count_ = result.count;

    LOG(INFO) << "[Match] total_pts=" << cloud->size() << " matched=" << result.count << " ratio=" << std::fixed
              << std::setprecision(3) << static_cast<float>(result.count) / cloud->size();

    // 求解 H * dx = b ， LDLT分解
    Eigen::Matrix<double, 6, 1> dx = result.H.ldlt().solve(result.b);

    // 平移更新
    state.p += dx.head<3>();

    // 旋转更新（SO3 指数映射）
    V3d dtheta = dx.tail<3>();
    double dthata_norm = dtheta.norm();

    // 使用 SO3指数映射更新旋转，比四元数近似更精确
    if (dthata_norm > 1e-10) {
      Qd dq = deltaQ(dtheta);
      state.q = (dq * state.q).normalized();
    } else {
      Qd dq(1.0, 0.5 * dtheta.x(), 0.5 * dtheta.y(), 0.5 * dtheta.z());
      state.q = (dq * state.q).normalized();
    }

    state.q.normalize();

    if (iter == MAX_ITER - 1) {
      inlier_count_ = 0;
      {
        const M3d R_final = state.q.toRotationMatrix();
        const V3d t_final = state.p;

        for (size_t i = 0; i < cloud->size(); ++i) {
          const auto& pt = cloud->points[i];
          const V3d p_lidar(pt.x, pt.y, pt.z);
          const V3d pw_final = R_final * p_lidar + t_final;

          PointType sp;
          sp.x = static_cast<float>(pw_final.x());
          sp.y = static_cast<float>(pw_final.y());
          sp.z = static_cast<float>(pw_final.z());

          PointType np;
          float d;

          if (!map->nearestSearch(sp, np, d)) {
            continue;
          }
          if (d > MAX_DIST) {
            continue;
          }
          V3d res = pw_final - V3d(np.x, np.y, np.z);
          double w = use_huber_ ? huberWeight(res.norm(), huber_k_) : 1.0;
          if (w > 0.5) {
            inlier_count_++;  // 计算内点数
          }
        }
        double sigma2_weighted = result.error / std::max(1, result.count - 6);

        // SVD 稳健求逆，截断小奇异值防止信息矩阵爆炸
        Eigen::JacobiSVD<Eigen::Matrix<double, 6, 6>> svd(result.H, Eigen::ComputeFullU | Eigen::ComputeFullV);
        auto singular_values = svd.singularValues();
        double max_sv = singular_values(0);
        constexpr double COND_THRESH = 1e8;
        Eigen::Vector<double, 6> inv_sv;
        for (int i = 0; i < 6; ++i) {
          if (singular_values(i) * COND_THRESH < max_sv) {
            inv_sv(i) = 0.0;  // 截断小奇异值
          } else {
            inv_sv(i) = 1.0 / singular_values(i);
          }
        }
        Eigen::Matrix<double, 6, 6> H_inv = svd.matrixV() * inv_sv.asDiagonal() * svd.matrixU().transpose();
        covariance_ = sigma2_weighted * H_inv;

        LOG(INFO) << "[Cov] sigma2=" << sigma2_weighted << " sv: " << singular_values.transpose()
                  << " cov_diag: " << covariance_.diagonal().transpose();
      }
    }

    LOG(INFO) << "iter = " << iter << " error = " << result.error / result.count << " count = " << result.count
              << " inlier = " << inlier_count_ << (use_huber_ ? " Huber " : "");

    if (dx.norm() < 1e-4) {
      break;
    }
  }

  return true;
}