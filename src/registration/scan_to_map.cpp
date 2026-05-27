#include "registration/scan_to_map.hpp"

#include <iostream>

ScanToMap::ScanToMap() {
  map_cloud_.reset(new pcl::PointCloud<PointXYZIT>());
  kdtree_.reset(new pcl::KdTreeFLANN<PointXYZIT>());
  q_w_curr_.setIdentity();
  t_w_curr_.setZero();
}

bool ScanToMap::fitPlane(const std::vector<Eigen::Vector3d>& points, Eigen::Vector4d& plane) {
  if (points.size() < 5) {
    return false;
  }

  Eigen::Vector3d center = Eigen::Vector3d::Zero();

  for (auto& p : points) {
    center += p;
  }

  center /= points.size();
  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();

  for (auto& p : points) {
    Eigen::Vector3d q = p - center;
    cov += q * q.transpose();
  }

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
  Eigen::Vector3d normal = solver.eigenvectors().col(0);
  double d = -normal.dot(center);
  plane.head<3>() = normal;
  plane[3] = d;
  return true;
}

void ScanToMap::optimizePose(pcl::PointCloud<PointXYZIT>::Ptr cloud) {
  if (map_cloud_->size() < 100) {
    return;
  }

  kdtree_->setInputCloud(map_cloud_);

  for (int iter = 0; iter < 5; ++iter) {
    Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> b = Eigen::Matrix<double, 6, 1>::Zero();
    int effective_num = 0;

    for (auto& pt : cloud->points) {
      Eigen::Vector3d p(pt.x, pt.y, pt.z);
      Eigen::Vector3d p_w = q_w_curr_ * p + t_w_curr_;
      PointXYZIT search_pt;
      search_pt.x = p_w.x();
      search_pt.y = p_w.y();
      search_pt.z = p_w.z();

      std::vector<int> indices(5);
      std::vector<float> dists(5);

      if (kdtree_->nearestKSearch(search_pt, 5, indices, dists) < 5) {
        continue;
      }

      std::vector<Eigen::Vector3d> near_points;

      for (auto idx : indices) {
        auto& mp = map_cloud_->points[idx];
        near_points.emplace_back(mp.x, mp.y, mp.z);
      }

      Eigen::Vector4d plane;

      if (!fitPlane(near_points, plane)) {
        continue;
      }

      Eigen::Vector3d n = plane.head<3>();
      double d = plane[3];
      double residual = n.dot(p_w) + d;

      if (std::fabs(residual) > 0.2) {
        continue;
      }

      Eigen::Matrix<double, 1, 6> J;
      Eigen::Matrix3d skew;
      skew << 0, -p_w.z(), p_w.y(), p_w.z(), 0, -p_w.x(), -p_w.y(), p_w.x(), 0;
      J.block<1, 3>(0, 0) = -n.transpose() * skew;
      J.block<1, 3>(0, 3) = n.transpose();
      H += J.transpose() * J;
      b += -J.transpose() * residual;
      effective_num++;
    }

    if (effective_num < 20) {
      continue;
    }

    Eigen::Matrix<double, 6, 1> dx = H.ldlt().solve(b);
    Eigen::Vector3d dtheta = dx.head<3>();
    Eigen::Vector3d dt = dx.tail<3>();
    double angle = dtheta.norm();
    Eigen::Quaterniond dq;

    if (angle < 1e-12) {
      dq.setIdentity();
    } else {
      dq = Eigen::Quaterniond(Eigen::AngleAxisd(angle, dtheta.normalized()));
    }

    q_w_curr_ = (dq * q_w_curr_).normalized();
    t_w_curr_ += dt;

    if (dx.norm() < 1e-4) {
      break;
    }
  }
}

void ScanToMap::addCloudToMap(pcl::PointCloud<PointXYZIT>::Ptr cloud) {
  pcl::PointCloud<PointXYZIT>::Ptr transformed(new pcl::PointCloud<PointXYZIT>());

  for (auto& pt : cloud->points) {
    Eigen::Vector3d p(pt.x, pt.y, pt.z);
    Eigen::Vector3d p_w = q_w_curr_ * p + t_w_curr_;
    PointXYZIT new_pt = pt;
    new_pt.x = p_w.x();
    new_pt.y = p_w.y();
    new_pt.z = p_w.z();
    transformed->push_back(new_pt);
  }

  *map_cloud_ += *transformed;
  pcl::VoxelGrid<PointXYZIT> voxel;
  voxel.setLeafSize(0.2f, 0.2f, 0.2f);
  voxel.setInputCloud(map_cloud_);
  pcl::PointCloud<PointXYZIT>::Ptr filtered(new pcl::PointCloud<PointXYZIT>());
  voxel.filter(*filtered);
  map_cloud_ = filtered;
}

void ScanToMap::process(pcl::PointCloud<PointXYZIT>::Ptr cloud) {
  optimizePose(cloud);
  addCloudToMap(cloud);
  std::cout << "pose = " << t_w_curr_.transpose() << std::endl;
}