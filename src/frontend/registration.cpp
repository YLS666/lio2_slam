#include "frontend/registration.hpp"
#include <glog/logging.h>
#include <tbb/tbb.h>
#include "utils/eigen_types.hpp"
#include "utils/math_types.hpp"

NDTRegistration::NDTRegistration(int max_iteration) : max_iteration_(max_iteration) {}

double NDTRegistration::huberWeight(double e) const {
  if (e <= huber_k_) {
    return 1.0;
  }
  return huber_k_ / e;
}

// IEKF 观测回调
// Jacobian (3×18) 非零块:
//   J = [I₃, 0, A, 0, 0, 0]  其中 A = -R·skew(q)
// 误差状态顺序: [δp(0:3), δv(3:6), δR(6:9), δbg(9:12), δba(12:15), δg(15:18)]
int NDTRegistration::computeResidualAndJacobians(const VoxelMap* map, const SE3& input_pose,
                                                 Eigen::Matrix<double, 18, 18>& HTVH,
                                                 Eigen::Matrix<double, 18, 1>& HTVr) {
  if (!source_ || source_->empty()) {
    LOG(WARNING) << "[NDT] source point cloud is empty";
    HTVH.setZero();
    HTVr.setZero();
    return 0;
  }

  if (!map || map->size() < 50) {
    LOG(WARNING) << "[NDT] map too small";
    HTVH.setZero();
    HTVr.setZero();
    return 0;
  }

  const double outlier_th = res_outlier_th_;
  const double ratio = info_ratio_;
  const size_t N = source_->size();

  struct NdtAccumulator {
    Eigen::Matrix<double, 18, 18> H;
    Eigen::Matrix<double, 18, 1> b;
    int effective = 0;
    const VoxelMap* map;
    const CloudPtr* source;
    SE3 pose;
    double outlier_th;

    NdtAccumulator(const VoxelMap* m, const CloudPtr* s, const SE3& p, double th)
        : map(m), source(s), pose(p), outlier_th(th) {
      H.setZero();
      b.setZero();
    }

    NdtAccumulator(NdtAccumulator& other, tbb::split)
        : map(other.map), source(other.source), pose(other.pose), outlier_th(other.outlier_th) {
      H.setZero();
      b.setZero();
      effective = 0;
    }

    void operator()(const tbb::blocked_range<size_t>& range) {
      const M3d pose_R = pose.so3().matrix();
      const V3d pose_t = pose.translation();

      for (size_t idx = range.begin(); idx != range.end(); ++idx) {
        const auto& pt = (*source)->points[idx];
        V3d q(pt.x, pt.y, pt.z);

        // 将点变换到世界坐标
        V3d qs = pose_R * q + pose_t;

        PointType query;
        query.x = static_cast<float>(qs.x());
        query.y = static_cast<float>(qs.y());
        query.z = static_cast<float>(qs.z());

        NDTCell cell;
        if (!map->getCell(query, cell, NearbyType::NEARBY6)) {
          continue;
        }
        if (!cell.ndt_estimated) {
          continue;
        }

        // 残差: e = qs - μ
        V3d e = qs - cell.mean;

        // 马氏距离离群检测
        double maha = e.transpose() * cell.info * e;
        if (float_check::isnan(maha) || maha > outlier_th) {
          continue;
        }

        // A = -R · skew(q)  (Jacobian 旋转块)
        M3d A = -pose_R * SO3::hat(q);

        const M3d& W = cell.info;
        M3d WA = W * A;
        V3d We = W * e;

        // H += Jᵀ·W·J  (仅 δp 和 δR 块非零)
        H.block<3, 3>(0, 0).noalias() += W;                   // δp-δp
        H.block<3, 3>(0, 6).noalias() += WA;                  // δp-δR
        H.block<3, 3>(6, 0).noalias() += WA.transpose();      // δR-δp
        H.block<3, 3>(6, 6).noalias() += A.transpose() * WA;  // δR-δR

        // b += -Jᵀ·W·e  (仅 δp 和 δR 块非零)
        b.segment<3>(0).noalias() -= We;                  // δp
        b.segment<3>(6).noalias() -= A.transpose() * We;  // δR

        effective++;
      }
    }

    void join(const NdtAccumulator& other) {
      H += other.H;
      b += other.b;
      effective += other.effective;
    }
  };

  NdtAccumulator acc(map, &source_, input_pose, outlier_th);

  int half_threads = static_cast<int>(std::max(1u, std::thread::hardware_concurrency()));
  tbb::task_arena arena(half_threads);
  arena.execute([&] { tbb::parallel_reduce(tbb::blocked_range<size_t>(0, N, 256), acc); });

  match_count_ = acc.effective;

  // 这使得 NDT 观测的等效噪声放大 100 倍，IMU 预测在 IEKF 中有更大权重
  // 这是保守策略，防止有偏差的 NDT 分布过度主导状态估计
  HTVH = acc.H * ratio;
  HTVr = acc.b * ratio;

  LOG(INFO) << "[NDT Obs] effective=" << acc.effective << " H_norm=" << acc.H.norm() << " b_norm=" << acc.b.norm();

  return acc.effective;
}
