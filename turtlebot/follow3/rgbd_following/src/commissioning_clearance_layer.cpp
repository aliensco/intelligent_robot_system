// Temporary, explicitly operator-confirmed clearance for commissioning only.
// This layer never clears a marked obstacle and never follows the robot.
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/cost_values.h>
#include <geometry_msgs/PolygonStamped.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <vector>

namespace rgbd_following {
class CommissioningClearanceLayer : public costmap_2d::Layer {
 public:
  void onInitialize() override {
    ros::NodeHandle nh("~/" + name_);
    std::string topic;
    nh.param("polygon_topic", topic, std::string("commissioning_clearance"));
    nh.param("lease_seconds", lease_, 10.0);
    if (!std::isfinite(lease_) || lease_ <= 0.0 || lease_ > 15.0)
      throw std::runtime_error("Clearance lease must be in (0, 15] seconds");
    enabled_ = true;
    current_ = false;
    subscriber_ = nh.subscribe(topic, 1, &CommissioningClearanceLayer::receive, this);
  }

  void updateBounds(double, double, double, double* min_x, double* min_y,
                    double* max_x, double* max_y) override {
    std::lock_guard<std::mutex> lock(mutex_);
    current_ = live();
    // Keep old bounds after expiry: the master resets these cells, allowing
    // the sensor layers to restore UNKNOWN instead of retaining stale free cells.
    if (have_bounds_) {
      *min_x = std::min(*min_x, min_x_);
      *min_y = std::min(*min_y, min_y_);
      *max_x = std::max(*max_x, max_x_);
      *max_y = std::max(*max_y, max_y_);
    }
  }

  void updateCosts(costmap_2d::Costmap2D& master, int min_i, int min_j,
                   int max_i, int max_j) override {
    std::lock_guard<std::mutex> lock(mutex_);
    current_ = live();
    if (!current_) return;
    const double half = master.getResolution() / 2.0;
    for (int j = min_j; j < max_j; ++j) {
      for (int i = min_i; i < max_i; ++i) {
        if (master.getCost(i, j) != costmap_2d::NO_INFORMATION) continue;
        double x, y;
        master.mapToWorld(i, j, x, y);
        // All four cell corners must be within the checked polygon. No
        // optimistic boundary rasterization or expansion into unchecked space.
        if (inside(x-half, y-half) && inside(x-half, y+half) &&
            inside(x+half, y-half) && inside(x+half, y+half))
          master.setCost(i, j, costmap_2d::FREE_SPACE);
      }
    }
  }

  void reset() override {
    std::lock_guard<std::mutex> lock(mutex_);
    active_ = false;
    current_ = false;
  }
  void deactivate() override { reset(); }

 private:
  bool live() const {
    const double wall_age = (ros::WallTime::now() - received_).toSec();
    const double age = (ros::Time::now() - stamp_).toSec();
    return active_ && wall_age >= 0.0 && wall_age <= lease_ && age >= 0.0 && age <= lease_;
  }

  bool inside(double x, double y) const {
    for (size_t i = 0; i < polygon_.size(); ++i) {
      const auto& a = polygon_[i];
      const auto& b = polygon_[(i+1) % polygon_.size()];
      if ((b.x-a.x)*(y-a.y) - (b.y-a.y)*(x-a.x) < -1e-8) return false;
    }
    return true;
  }

  void receive(const geometry_msgs::PolygonStampedConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    const double age = (ros::Time::now() - msg->header.stamp).toSec();
    if (msg->header.frame_id != layered_costmap_->getGlobalFrameID() ||
        msg->header.stamp.isZero() || msg->header.stamp <= stamp_ ||
        age < 0.0 || age > 0.5 || msg->polygon.points.size() != 4) {
      ROS_WARN_THROTTLE(2.0, "Ignoring stale/wrong-frame/invalid commissioning polygon");
      return;
    }
    auto points = msg->polygon.points;
    double lo_x = INFINITY, lo_y = INFINITY, hi_x = -INFINITY, hi_y = -INFINITY, area = 0.0;
    for (size_t i = 0; i < points.size(); ++i) {
      const auto& a = points[i];
      const auto& b = points[(i+1) % points.size()];
      const auto& c = points[(i+2) % points.size()];
      if (!std::isfinite(a.x) || !std::isfinite(a.y) || !std::isfinite(a.z) ||
          std::abs(a.z) > 0.01 ||
          (b.x-a.x)*(c.y-b.y) - (b.y-a.y)*(c.x-b.x) <= 0.0) return;
      area += a.x*b.y - a.y*b.x;
      lo_x = std::min(lo_x, double(a.x)); lo_y = std::min(lo_y, double(a.y));
      hi_x = std::max(hi_x, double(a.x)); hi_y = std::max(hi_y, double(a.y));
    }
    if (area < 0.02 || area > 8.0 || hi_x-lo_x > 2.0 || hi_y-lo_y > 2.0) return;
    polygon_ = points;
    stamp_ = msg->header.stamp;
    received_ = ros::WallTime::now();
    active_ = true;
    if (!have_bounds_) { min_x_ = lo_x; min_y_ = lo_y; max_x_ = hi_x; max_y_ = hi_y; }
    else {
      min_x_ = std::min(min_x_, lo_x); min_y_ = std::min(min_y_, lo_y);
      max_x_ = std::max(max_x_, hi_x); max_y_ = std::max(max_y_, hi_y);
    }
    have_bounds_ = true;
    ROS_INFO("Accepted operator-confirmed clearance for %.1f seconds", lease_);
  }

  ros::Subscriber subscriber_;
  std::mutex mutex_;
  std::vector<geometry_msgs::Point32> polygon_;
  ros::Time stamp_;
  ros::WallTime received_;
  double lease_ = 10.0, min_x_ = 0, min_y_ = 0, max_x_ = 0, max_y_ = 0;
  bool active_ = false, have_bounds_ = false;
};
}
PLUGINLIB_EXPORT_CLASS(rgbd_following::CommissioningClearanceLayer, costmap_2d::Layer)
