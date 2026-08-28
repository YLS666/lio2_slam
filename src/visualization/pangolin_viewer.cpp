#include "visualization/pangolin_viewer.hpp"

#include <glog/logging.h>
#include <pangolin/gl/gl.h>
#include <pangolin/gl/opengl_render_state.h>
#include <pcl/common/transforms.h>
#include <chrono>
#include <thread>
#include "cloud_utils/point_type.hpp"
#include "visualization/viewer_control.hpp"
#include "visualization/viewer_gl_utils.hpp"
#include "visualization/viewer_render.hpp"

PangolinViewer::PangolinViewer() {
  current_cloud_.reset(new PointCloudType());
  raw_current_cloud_.reset(new PointCloudType());
  local_map_.reset(new PointCloudType());
  global_map_.reset(new PointCloudType());

  LOG(INFO) << "PangolinViewer constructed";
}

PangolinViewer::~PangolinViewer() { stop(); }

void PangolinViewer::start() {
  if (running_.load()) {
    LOG(WARNING) << "Viewer already running";
    return;
  }

  should_exit_.store(false);
  initialized_.store(false);
  running_.store(true);

  thread_ = std::make_unique<std::thread>(&PangolinViewer::run, this);

  // 等待渲染线程完成 Pangolin 初始化（不调用任何 Pangolin API）
  while (running_.load() && !initialized_.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  if (!initialized_.load()) {
    LOG(ERROR) << "PangolinViewer 初始化失败";
    running_.store(false);
  } else {
    LOG(INFO) << "PangolinViewer started";
  }
}

void PangolinViewer::stop() {
  if (!running_.load()) {
    return;
  }

  should_exit_.store(true);

  if (thread_ && thread_->joinable()) {
    thread_->join();
  }

  thread_.reset();
  running_.store(false);

  LOG(INFO) << "PangolinViewer stopped";
}

void PangolinViewer::updateCurrentCloud(const CloudPtr& cloud, const Eigen::Matrix4f& T) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  raw_current_cloud_ = cloud;  // shared_ptr 赋值, O(1)
  current_T_ = T;
  current_cloud_dirty_ = true;
}

void PangolinViewer::updateLocalMap(const CloudPtr& cloud) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  *local_map_ = *cloud;
  local_map_dirty_ = true;
}

void PangolinViewer::appendGlobalMap(const CloudPtr& cloud) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  *global_map_ += *cloud;
  CloudPtr temp(new PointCloudType());
  temp = dsCloud(global_map_, 1.0f);
  global_map_.swap(temp);
  global_map_dirty_ = true;
}

void PangolinViewer::clearGlobalMap() {
  std::lock_guard<std::mutex> lock(data_mutex_);
  global_map_->clear();
  global_map_dirty_ = true;
}

void PangolinViewer::ResetCamera() {
  if (camera_) {
    camera_->SetModelViewMatrix(default_view_);
  }
}

void PangolinViewer::FollowRobot() {
  V3d p;
  Qd q;

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    p = current_position_;
    q = current_orientation_;
  }

  double yaw = std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));

  constexpr double follow_distance = 30.0;
  constexpr double camera_height_slam = 100.0;  // slam Z高度

  // ========= SLAM世界坐标系 =========
  // 局部偏移：机器人本体坐标系，相机在机器人负前方（身后）
  Eigen::Vector3d local_offset(follow_distance, 0.0, 0.0);

  // yaw旋转矩阵：绕slam Z轴旋转局部偏移，转到世界坐标系
  Eigen::Matrix3d Rz;
  Rz << std::cos(yaw), -std::sin(yaw), 0.0, std::sin(yaw), std::cos(yaw), 0.0, 0.0, 0.0, 1.0;

  // 相机世界位置：机器人位置 + 旋转后的偏移，叠加高度
  Eigen::Vector3d cam_slam = p + Rz * local_offset;
  cam_slam.z() = camera_height_slam;

  // slam -> GL映射规则：gl.X = slam.X; gl.Y = slam.Z; gl.Z = slam.Y
  Eigen::Vector3d cam_gl(cam_slam.x(), cam_slam.z(), cam_slam.y());
  Eigen::Vector3d target_gl(p.x(), p.z(), p.y());  // 直接看向机器人（世界点）

  // up向量：【世界固定向上，永远不变0,0,-1，绝对不能变】
  camera_->SetModelViewMatrix(pangolin::ModelViewLookAt(cam_gl.x(), cam_gl.y(), cam_gl.z(), target_gl.x(),
                                                        target_gl.y(), target_gl.z(), 0, 0, -1));
}

void PangolinViewer::updatePose(const V3d& p, const Qd& q, double timestamp) {
  std::lock_guard<std::mutex> lock(data_mutex_);

  current_position_ = p;
  current_orientation_ = q;

  trajectory_.emplace_back(timestamp, p, q);

  constexpr size_t kMaxTrajectory = 100000;

  while (trajectory_.size() > kMaxTrajectory) {
    trajectory_.pop_front();
  }
}

void PangolinViewer::updateMotionInfo(const V3d& vel, const V3d& gyr_raw, const V3d& acc_raw) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  vel_magnitude_ = static_cast<float>(vel.norm());
  gyr_magnitude_ = static_cast<float>(gyr_raw.norm());
  acc_magnitude_ = static_cast<float>(acc_raw.norm());
}

void PangolinViewer::run() {
  try {
    constexpr int kWindowWidth = 1920;
    constexpr int kWindowHeight = 1080;
    const int view_w = static_cast<int>(kWindowWidth * 0.75f);  // 3D View 占窗口宽度的 75%
    const int view_h = kWindowHeight;

    // ---------- 初始化 Pangolin ----------
    pangolin::CreateWindowAndBind("LIO2-SLAM Viewer", kWindowWidth, kWindowHeight);

    pangolin::CreatePanel("ui").SetBounds(0.7, 1.0, 0.75, 1.0);

    control_ = std::make_shared<ViewerControl>();

    // 深度测试
    glEnable(GL_DEPTH_TEST);

    // 混合 → 透明点云
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // 相机参数（GL 坐标系：y 轴朝上，从上方 100m 俯视）
    camera_ = std::make_unique<pangolin::OpenGlRenderState>(
        pangolin::ProjectionMatrix(view_w, view_h, 420.0, 420.0, view_w / 2.0, view_h / 2.0, 0.1, 1000.0),
        pangolin::ModelViewLookAt(0.0, 100.0, 0.0, 0.0, 0.0, 0.0, pangolin::AxisZ));

    default_view_ = camera_->GetModelViewMatrix();

    // ---------- 初始化子模块 ----------
    layout_.Init(*camera_);
    plot_.Init();

    // 通知主线程：初始化完成
    initialized_.store(true);

    // 设置背景色
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    // ---------- 本地缓存（减少锁持有时间）----------
    viewer::ViewerCache cache;

    // ---------- 主循环 ----------
    while (!should_exit_.load() && !pangolin::ShouldQuit()) {
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

      if (control_->ResetView()) {
        ResetCamera();
        control_->DisableFollow();
      }

      if (control_->FollowRobot()) {
        FollowRobot();
      }

      if (control_->ClearTrajectory()) {
        trajectory_.clear();
      }

      // ========== 获取数据（短期锁）==========
      {
        std::lock_guard<std::mutex> lock(data_mutex_);

        if (current_cloud_dirty_) {
          pcl::transformPointCloud(*raw_current_cloud_, *cache.current_cloud, current_T_);  // 变换移到渲染线程
          current_cloud_dirty_ = false;
        }

        if (local_map_dirty_) {
          *cache.local_map = *local_map_;
          local_map_dirty_ = false;
        }

        if (global_map_dirty_) {
          *cache.global_map = *global_map_;
          global_map_dirty_ = false;
        }

        cache.trajectory = trajectory_;
      }

      // ========== 激活 3D View 并渲染 ==========
      layout_.Scene().Activate(*camera_);

      // 网格地面 (GL 坐标系，y=0 水平面，灰色)
      glLineWidth(1.0f);
      glColor3f(0.5f, 0.5f, 0.5f);
      glBegin(GL_LINES);
      constexpr float kGridSize = 20.0f;
      constexpr int kGridHalf = 20;
      for (int i = -kGridHalf; i <= kGridHalf; ++i) {
        float coord = i * kGridSize;
        // 沿 X 方向的线（z 变化）
        glVertex3f(coord, 0.0f, -kGridHalf * kGridSize);
        glVertex3f(coord, 0.0f, kGridHalf * kGridSize);
        // 沿 Z 方向的线（x 变化）
        glVertex3f(-kGridHalf * kGridSize, 0.0f, coord);
        glVertex3f(kGridHalf * kGridSize, 0.0f, coord);
      }
      glEnd();

      // 渲染各层点云
      if (control_->ShowGlobalMap()) {
        viewer::DrawGlobalMap(cache.global_map);
      }
      if (control_->ShowLocalMap()) {
        viewer::DrawLocalMap(cache.local_map);
      }
      if (control_->ShowScan()) {
        viewer::DrawCurrentCloud(cache.current_cloud);
      }

      // 渲染轨迹
      viewer::DrawTrajectory(cache.trajectory);

      // ========== HUD ==========
      viewer::HudInfo hud;
      hud.velocity = vel_magnitude_;
      hud.gyroscope = gyr_magnitude_;
      hud.acceleration = acc_magnitude_;
      hud.trajectory_size = cache.trajectory.size();
      hud.global_map_size = cache.global_map->size();

      viewer::DrawHUD(hud);

      // ========== Plot 渲染 ==========
      plot_.Push(vel_magnitude_, gyr_magnitude_, acc_magnitude_);
      plot_.RenderVel();
      plot_.RenderGyr();
      plot_.RenderAcc();

      // ========== 交换缓冲 ==========
      pangolin::FinishFrame();

      // 帧率控制 ~30 FPS（降低 CPU 占用）
      std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }
  } catch (const std::exception& e) {
    LOG(ERROR) << "PangolinViewer 异常: " << e.what();
    initialized_.store(false);
  } catch (...) {
    LOG(ERROR) << "PangolinViewer 未知异常";
    initialized_.store(false);
  }

  LOG(INFO) << "PangolinViewer main loop exited";
}

Eigen::Vector3f PangolinViewer::heightToColor(float z, float z_min, float z_max) {
  return viewer::HeightToColor(z, z_min, z_max);
}