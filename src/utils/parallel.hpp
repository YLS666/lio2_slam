#pragma once

#include <tbb/blocked_range.h>
#include <tbb/concurrent_unordered_map.h>
#include <tbb/enumerable_thread_specific.h>
#include <tbb/parallel_for.h>
#include <tbb/task_arena.h>
#include <tbb/tbb.h>
#include <thread>

namespace lio {

// 解析线程数:
//   <=0 -> 不限制, 使用全部硬件线程 (离线建图默认, 拉满性能)
//   > 0 -> 显式限制线程数 (未来实时定位时预留 CPU 给其他任务)
inline int effectiveThreads(int num_threads) {
  if (num_threads > 0) {
    return num_threads;
  }
  unsigned int hc = std::thread::hardware_concurrency();
  return static_cast<int>(hc == 0 ? 1 : hc);
}

}  // namespace lio
