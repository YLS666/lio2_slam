#include <g2o/core/base_multi_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

// 数据结构
struct PathPoint {
  double x = 0.0;
  double y = 0.0;
};

struct Obstacle {
  double x = 0.0;
  double y = 0.0;
};

// 参数
namespace config {
// 路径点数量
constexpr int kPathPointCount = 21;
// 起点
constexpr double kStartX = 0.0;
constexpr double kStartY = 0.0;
// 终点
constexpr double kEndX = 10.0;
constexpr double kEndY = 0.0;

// 障碍物
constexpr double kObstacleX = 5.0;
constexpr double kObstacleY = 0.0;
// 障碍物安全半径
constexpr double kSafeRadius = 0.5;
// 障碍物影响范围
constexpr double kInfluenceRadius = 2.0;
// 绕障轨迹最大侧向距离
// 0.65m 表示：
// 安全半径 0.5m + 约 0.15m 的余量
constexpr double kDetourAmplitude = 0.65;

// 权重
// 原始曲线约束
constexpr double kPriorWeight = 1.0;
// 平滑约束
constexpr double kSmoothWeight = 20.0;
// 障碍物约束
constexpr double kObstacleWeight = 200.0;
// 绕障方向参考约束
constexpr double kDetourWeight = 0.5;

// 优化迭代次数
constexpr int kIterations = 100;

// 输出目录
const char* kOutputDir = "./g2o_obstacle_output";

}  // namespace config

// 工具函数
double DistanceSquared(const Eigen::Vector2d& p, const Eigen::Vector2d& q) {
  const double dx = p.x() - q.x();
  const double dy = p.y() - q.y();

  return dx * dx + dy * dy;
}

// Path Vertex
class PathVertex : public g2o::BaseVertex<2, Eigen::Vector2d> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void setToOriginImpl() override { _estimate.setZero(); }

  void oplusImpl(const double* update) override {
    _estimate.x() += update[0];
    _estimate.y() += update[1];
  }

  bool read(std::istream& /*is*/) override { return false; }

  bool write(std::ostream& /*os*/) const override { return false; }
};

// 1. 原始曲线约束
// 作用：
// 让优化后的路径尽量保持原始轨迹。
// 但是障碍物附近权重可以降低。
// error:
//     x - x_original
//     y - y_original

class PriorEdge : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, PathVertex> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void computeError() override {
    const auto* vertex = static_cast<const PathVertex*>(_vertices[0]);
    const Eigen::Vector2d& p = vertex->estimate();

    _error[0] = p.x() - _measurement.x();
    _error[1] = p.y() - _measurement.y();
  }

  void linearizeOplus() override {
    _jacobianOplusXi.setZero();

    _jacobianOplusXi(0, 0) = 1.0;
    _jacobianOplusXi(1, 1) = 1.0;
  }

  bool read(std::istream& /*is*/) override { return false; }

  bool write(std::ostream& /*os*/) const override { return false; }
};

// 2. 障碍物约束
// 障碍物为一个点。
// 安全半径：0.5m
// 当：distance >= 0.5   error = 0
// 当：distance < 0.5   error = safe_radius^2 - distance^2
// 这样可以强制路径离开障碍物。
class ObstacleEdge : public g2o::BaseUnaryEdge<1, Eigen::Vector2d, PathVertex> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void computeError() override {
    const auto* vertex = static_cast<const PathVertex*>(_vertices[0]);
    const Eigen::Vector2d& p = vertex->estimate();

    const double dx = p.x() - _measurement.x();
    const double dy = p.y() - _measurement.y();
    const double dist2 = dx * dx + dy * dy;
    const double safe2 = config::kSafeRadius * config::kSafeRadius;

    if (dist2 >= safe2) {
      _error[0] = 0.0;
    } else {
      _error[0] = safe2 - dist2;
    }
  }

  void linearizeOplus() override {
    const auto* vertex = static_cast<const PathVertex*>(_vertices[0]);
    const Eigen::Vector2d& p = vertex->estimate();

    const double dx = p.x() - _measurement.x();
    const double dy = p.y() - _measurement.y();
    const double dist2 = dx * dx + dy * dy;
    const double safe2 = config::kSafeRadius * config::kSafeRadius;

    _jacobianOplusXi.setZero();

    if (dist2 >= safe2) {
      return;
    }

    _jacobianOplusXi(0, 0) = -2.0 * dx;
    _jacobianOplusXi(0, 1) = -2.0 * dy;
  }

  bool read(std::istream& /*is*/) override { return false; }

  bool write(std::ostream& /*os*/) const override { return false; }
};

// 3. 平滑约束
// 使用二阶差分：
// P(i-1) - 2P(i) + P(i+1) = 0
// 约束 x 和 y 两个方向。
class SmoothXEdge : public g2o::BaseMultiEdge<1, double> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SmoothXEdge() { resize(3); }

  void computeError() override {
    const auto* p0 = static_cast<const PathVertex*>(_vertices[0]);
    const auto* p1 = static_cast<const PathVertex*>(_vertices[1]);
    const auto* p2 = static_cast<const PathVertex*>(_vertices[2]);

    _error[0] = p0->estimate().x() - 2.0 * p1->estimate().x() + p2->estimate().x();
  }

  void linearizeOplus() override {
    _jacobianOplus[0].setZero();
    _jacobianOplus[1].setZero();
    _jacobianOplus[2].setZero();

    _jacobianOplus[0](0, 0) = 1.0;
    _jacobianOplus[1](0, 0) = -2.0;
    _jacobianOplus[2](0, 0) = 1.0;
  }

  bool read(std::istream&) override { return false; }

  bool write(std::ostream&) const override { return false; }
};

class SmoothYEdge : public g2o::BaseMultiEdge<1, double> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SmoothYEdge() { resize(3); }

  void computeError() override {
    const auto* p0 = static_cast<const PathVertex*>(_vertices[0]);
    const auto* p1 = static_cast<const PathVertex*>(_vertices[1]);
    const auto* p2 = static_cast<const PathVertex*>(_vertices[2]);

    _error[0] = p0->estimate().y() - 2.0 * p1->estimate().y() + p2->estimate().y();
  }

  void linearizeOplus() override {
    _jacobianOplus[0].setZero();
    _jacobianOplus[1].setZero();
    _jacobianOplus[2].setZero();

    _jacobianOplus[0](0, 1) = 1.0;
    _jacobianOplus[1](0, 1) = -2.0;
    _jacobianOplus[2](0, 1) = 1.0;
  }

  bool read(std::istream&) override { return false; }

  bool write(std::ostream&) const override { return false; }
};

// 4. 绕障方向约束
// 这个 Edge 不是用来保证安全距离的。
// 它只负责解决： 左绕和右绕完全对称的问题。
// 在障碍物影响范围内，给路径一个很弱的侧向参考。
// 距离障碍物越近，参考权重越大。
class DetourEdge : public g2o::BaseUnaryEdge<1, double, PathVertex> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit DetourEdge(double obstacle_x, double obstacle_y, double side)
      : obstacle_x_(obstacle_x), obstacle_y_(obstacle_y), side_(side) {}

  void computeError() override {
    const auto* vertex = static_cast<const PathVertex*>(_vertices[0]);
    const Eigen::Vector2d& p = vertex->estimate();

    // 这里的 measurement 是当前点希望达到的 y。
    //
    // 但是 target 不是一个固定的整段偏移，
    // 而是在障碍物影响范围内形成 sin^2 曲线。
    const double target_y = ComputeTargetY(p.x());

    _error[0] = p.y() - target_y;
  }

  void linearizeOplus() override {
    const auto* vertex = static_cast<const PathVertex*>(_vertices[0]);
    const Eigen::Vector2d& p = vertex->estimate();

    _jacobianOplusXi.setZero();
    _jacobianOplusXi(0, 1) = 1.0;

    (void)p;
  }

  bool read(std::istream& /*is*/) override { return false; }

  bool write(std::ostream& /*os*/) const override { return false; }

 private:
  double ComputeTargetY(double x) const {
    const double x0 = obstacle_x_ - config::kInfluenceRadius;
    const double x1 = obstacle_x_ + config::kInfluenceRadius;

    // 不在影响范围内
    if (x <= x0 || x >= x1) {
      return obstacle_y_;
    }

    const double t = (x - x0) / (x1 - x0);

    // sin^2：
    // t = 0 -> 0
    // t = 0.5 -> 1
    // t = 1 -> 0
    // 所以路径： 原始轨迹 -> 平滑离开 -> 障碍物附近最大偏移 -> 平滑回来
    const double s = std::sin(M_PI * t);
    const double offset = config::kDetourAmplitude * s * s;

    return obstacle_y_ + side_ * offset;
  }

 private:
  double obstacle_x_;
  double obstacle_y_;

  // +1 = 左绕
  // -1 = 右绕
  double side_;
};

// 创建原始直线路径
std::vector<PathPoint> CreatePath() {
  std::vector<PathPoint> path;
  path.reserve(config::kPathPointCount);

  for (int i = 0; i < config::kPathPointCount; ++i) {
    const double ratio = static_cast<double>(i) / static_cast<double>(config::kPathPointCount - 1);

    PathPoint point;
    point.x = config::kStartX + ratio * (config::kEndX - config::kStartX);
    point.y = config::kStartY + ratio * (config::kEndY - config::kStartY);
    path.push_back(point);
  }

  return path;
}

// 创建障碍物
Obstacle CreateObstacle() {
  Obstacle obstacle;
  obstacle.x = config::kObstacleX;
  obstacle.y = config::kObstacleY;

  return obstacle;
}

// 保存 CSV
void SavePath(const std::string& file, const std::vector<PathPoint>& path) {
  std::ofstream ofs(file);

  if (!ofs.is_open()) {
    std::cerr << "Failed to open: " << file << std::endl;
    return;
  }

  ofs << "index,x,y\n";

  for (size_t i = 0; i < path.size(); ++i) {
    ofs << i << "," << path[i].x << "," << path[i].y << "\n";
  }
}

// 保存障碍物
void SaveObstacle(const std::string& file, const Obstacle& obstacle) {
  std::ofstream ofs(file);

  if (!ofs.is_open()) {
    std::cerr << "Failed to open: " << file << std::endl;
    return;
  }

  ofs << "x,y,radius\n";
  ofs << obstacle.x << "," << obstacle.y << "," << config::kSafeRadius << "\n";
}

// 计算路径距离障碍物的最小值
double ComputeMinObstacleDistance(const std::vector<PathPoint>& path, const Obstacle& obstacle) {
  double min_distance = std::numeric_limits<double>::max();
  for (const auto& point : path) {
    const double dx = point.x - obstacle.x;
    const double dy = point.y - obstacle.y;
    const double distance = std::sqrt(dx * dx + dy * dy);

    min_distance = std::min(min_distance, distance);
  }

  return min_distance;
}

// 打印路径
void PrintPath(const std::string& title, const std::vector<PathPoint>& path) {
  std::cout << "\n========== " << title << " ==========\n";

  for (size_t i = 0; i < path.size(); ++i) {
    std::cout << "P" << i << " : "
              << "(" << path[i].x << ", " << path[i].y << ")"
              << "\n";
  }
}

int main() {
  namespace fs = std::filesystem;

  // 1. 创建测试数据
  const auto original_path = CreatePath();
  const Obstacle obstacle = CreateObstacle();
  std::vector<PathPoint> optimized_path = original_path;

  // 2. 创建输出目录
  fs::create_directories(config::kOutputDir);
  SavePath(std::string(config::kOutputDir) + "/before.csv", original_path);
  SaveObstacle(std::string(config::kOutputDir) + "/obstacle.csv", obstacle);

  // 3. g2o 类型
  using BlockSolver = g2o::BlockSolver<g2o::BlockSolverTraits<2, 2>>;
  using LinearSolver = g2o::LinearSolverDense<BlockSolver::PoseMatrixType>;
  auto linear_solver = std::make_unique<LinearSolver>();
  auto block_solver = std::make_unique<BlockSolver>(std::move(linear_solver));
  auto* algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));

  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(algorithm);
  optimizer.setVerbose(true);

  // 4. 添加 Path Vertex
  std::vector<PathVertex*> vertices;
  vertices.reserve(original_path.size());
  for (size_t i = 0; i < original_path.size(); ++i) {
    auto* vertex = new PathVertex();
    vertex->setId(static_cast<int>(i));
    vertex->setEstimate(Eigen::Vector2d(original_path[i].x, original_path[i].y));

    // 起点和终点固定
    if (i == 0 || i == original_path.size() - 1) {
      vertex->setFixed(true);
    }

    optimizer.addVertex(vertex);
    vertices.push_back(vertex);
  }

  // 5. 原始曲线约束
  // 所有点都有 prior。
  // 但是障碍物附近降低 prior，
  // 允许它们为了绕障而偏离原始路径。
  int edge_id = 0;
  for (size_t i = 1; i + 1 < original_path.size(); ++i) {
    const Eigen::Vector2d original(original_path[i].x, original_path[i].y);

    auto* edge = new PriorEdge();
    edge->setId(edge_id++);
    edge->setVertex(0, vertices[i]);
    edge->setMeasurement(original);

    // 根据当前点距离障碍物的距离，
    // 动态降低障碍物附近的 prior。
    //
    // distance >= influence：
    //     prior = 1.0
    //
    // distance -> 0：
    //     prior -> 0
    const double dx = original.x() - obstacle.x;
    const double dy = original.y() - obstacle.y;
    const double distance = std::sqrt(dx * dx + dy * dy);
    double weight = config::kPriorWeight;

    if (distance < config::kInfluenceRadius) {
      const double ratio = distance / config::kInfluenceRadius;
      // 障碍物附近 prior 逐渐降低
      weight *= ratio * ratio;
    }

    Eigen::Matrix2d information = Eigen::Matrix2d::Identity() * weight;
    edge->setInformation(information);
    optimizer.addEdge(edge);
  }

  // 6. 平滑约束
  // 分成x约束和y约束
  for (size_t i = 1; i + 1 < vertices.size(); ++i) {
    auto* edge_x = new SmoothXEdge();
    edge_x->setId(edge_id++);
    edge_x->setVertex(0, vertices[i - 1]);
    edge_x->setVertex(1, vertices[i]);
    edge_x->setVertex(2, vertices[i + 1]);
    edge_x->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * config::kSmoothWeight);
    optimizer.addEdge(edge_x);

    auto* edge_y = new SmoothYEdge();
    edge_y->setId(edge_id++);
    edge_y->setVertex(0, vertices[i - 1]);
    edge_y->setVertex(1, vertices[i]);
    edge_y->setVertex(2, vertices[i + 1]);
    edge_y->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * config::kSmoothWeight);
    optimizer.addEdge(edge_y);
  }

  // 7. 障碍物约束
  // 只有原始路径距离障碍物小于 influence radius
  // 的点才添加 obstacle edge。
  const double influence2 = config::kInfluenceRadius * config::kInfluenceRadius;
  for (size_t i = 1; i + 1 < original_path.size(); ++i) {
    const Eigen::Vector2d p(original_path[i].x, original_path[i].y);
    const Eigen::Vector2d o(obstacle.x, obstacle.y);

    if (DistanceSquared(p, o) > influence2) {
      continue;
    }

    auto* edge = new ObstacleEdge();
    edge->setId(edge_id++);
    edge->setVertex(0, vertices[i]);
    edge->setMeasurement(o);
    edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * config::kObstacleWeight);
    optimizer.addEdge(edge);
  }

  // 8. 绕障方向
  // +1 = 左绕
  // -1 = 右绕
  // 这里测试左绕。
  // 如果想测试右绕，改成 -1.0。
  constexpr double kDetourSide = +1.0;
  for (size_t i = 1; i + 1 < original_path.size(); ++i) {
    const Eigen::Vector2d p(original_path[i].x, original_path[i].y);
    const Eigen::Vector2d o(obstacle.x, obstacle.y);
    const double distance = std::sqrt(DistanceSquared(p, o));

    if (distance > config::kInfluenceRadius) {
      continue;
    }

    auto* edge = new DetourEdge(obstacle.x, obstacle.y, kDetourSide);
    edge->setId(edge_id++);
    edge->setVertex(0, vertices[i]);
    edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * config::kDetourWeight);
    optimizer.addEdge(edge);
  }

  // 9. 初始化
  optimizer.initializeOptimization();

  // 10. 优化
  std::cout << "========== Optimization Start ==========\n";
  optimizer.optimize(config::kIterations);
  std::cout << "========== Optimization Finish ==========\n";

  // 11. 获取优化结果
  for (size_t i = 0; i < vertices.size(); ++i) {
    const Eigen::Vector2d& p = vertices[i]->estimate();
    optimized_path[i].x = p.x();
    optimized_path[i].y = p.y();
  }

  // 12. 保存结果
  SavePath(std::string(config::kOutputDir) + "/after.csv", optimized_path);

  // 13. 输出结果
  PrintPath("Original Path", original_path);
  PrintPath("Optimized Path", optimized_path);

  // 14. 计算最小障碍物距离
  const double before_min_distance = ComputeMinObstacleDistance(original_path, obstacle);
  const double after_min_distance = ComputeMinObstacleDistance(optimized_path, obstacle);

  std::cout << "\nObstacle:\n"
            << "  x = " << obstacle.x << "\n"
            << "  y = " << obstacle.y << "\n";
  std::cout << "\nSafe Radius = " << config::kSafeRadius << " m\n";
  std::cout << "\nMinimum distance before optimization: " << before_min_distance << " m\n";
  std::cout << "Minimum distance after optimization:  " << after_min_distance << " m\n";
  std::cout << "\nOutput:\n"
            << "  " << config::kOutputDir << "/before.csv\n";
  std::cout << "  " << config::kOutputDir << "/after.csv\n";
  std::cout << "  " << config::kOutputDir << "/obstacle.csv\n";

  return 0;
}