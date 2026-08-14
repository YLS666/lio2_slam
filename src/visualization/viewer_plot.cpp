#include "visualization/viewer_plot.hpp"

namespace viewer {

void ViewerPlot::Init() {
  log_ = std::make_unique<pangolin::DataLog>();

  log_->SetLabels({
      "Time",
      "Velocity",
      "Gyroscope",
      "Acceleration",
  });

  //---------------- Velocity ----------------//

  vel_plot_ = new pangolin::Plotter(log_.get(), 0.0f, kHistory, 0.0f, kVelMax);

  vel_plot_->SetBackgroundColour(pangolin::Colour(0.08f, 0.08f, 0.10f));

  vel_plot_->AddSeries("$0", "$1", pangolin::DrawingModeLine, pangolin::Colour(1.0f, 0.55f, 0.0f), "Velocity");

  //---------------- Gyroscope ----------------//

  gyr_plot_ = new pangolin::Plotter(log_.get(), 0.0f, kHistory, 0.0f, kGyrMax);

  gyr_plot_->SetBackgroundColour(pangolin::Colour(0.08f, 0.08f, 0.10f));

  gyr_plot_->AddSeries("$0", "$2", pangolin::DrawingModeLine, pangolin::Colour(0.2f, 0.7f, 1.0f), "Gyroscope");

  //---------------- Acceleration ----------------//

  acc_plot_ = new pangolin::Plotter(log_.get(), 0.0f, kHistory, 0.0f, kAccMax);

  acc_plot_->SetBackgroundColour(pangolin::Colour(0.08f, 0.08f, 0.10f));

  acc_plot_->AddSeries("$0", "$3", pangolin::DrawingModeLine, pangolin::Colour(1.0f, 0.2f, 0.2f), "Acceleration");
}

void ViewerPlot::RenderVel() {
  if (vel_plot_) {
    pangolin::Display("plot_vel").Activate();
    vel_plot_->Render();
  }
}

void ViewerPlot::RenderGyr() {
  if (gyr_plot_) {
    pangolin::Display("plot_gyr").Activate();
    gyr_plot_->Render();
  }
}

void ViewerPlot::RenderAcc() {
  if (acc_plot_) {
    pangolin::Display("plot_acc").Activate();
    acc_plot_->Render();
  }
}

void ViewerPlot::Push(float vel, float gyr, float acc) {
  if (!log_) {
    return;
  }

  time_ += 0.1f;

  log_->Log(time_, vel, gyr, acc);
}

}  // namespace viewer