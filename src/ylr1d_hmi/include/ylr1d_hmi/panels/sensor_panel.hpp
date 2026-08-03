#ifndef YLR1D_HMI__SENSOR_PANEL_HPP_
#define YLR1D_HMI__SENSOR_PANEL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <QWidget>
#include <QLabel>
#include <QImage>
#include <QString>
#include <QTimer>
#include <QTabWidget>
#include <QComboBox>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "ylr1d_hmi/panels/topic_status.hpp"

namespace ylr1d_hmi {

/// Sensor visualization panel.
/// Subscribes to every sensor on the robot and renders each type in its
/// best-suited form: cameras (RGB/depth/IR + camera_info params), depth
/// camera point clouds (2-D projection), radar & ultrasonic (laser scan
/// views + statistics), IMU (numeric readout). Every sensor also shows a
/// TopicStatus line (last update time, frame count, observed rate).
///
/// Threading: ROS callbacks run on the GUI thread (HmiWindow::onRosSpin
/// calls spin_some inside a QTimer), so callbacks only cache the newest
/// message and a low-rate QTimer renders from the cached frames.
class SensorPanel : public QWidget {
  Q_OBJECT

public:
  explicit SensorPanel(rclcpp::Node::SharedPtr node, QWidget * parent = nullptr);

private:
  rclcpp::Node::SharedPtr node_;
  QTimer * refresh_timer_{nullptr};
  // Keep all subscriptions alive (they die if the SharedPtr is dropped)
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;

  // ── Cameras: 3 units (Global / Left / Right) × (rgb, depth, ir) + info ──
  // Subscribes to all three, but only the selected camera's streams are
  // rendered (WSL 软渲染下不能全量渲染，切换查看避免压死渲染管线).
  struct CameraGroup {
    QString name;
    int update_rate{30};
    sensor_msgs::msg::Image::SharedPtr rgb;
    sensor_msgs::msg::Image::SharedPtr depth;
    sensor_msgs::msg::Image::SharedPtr ir;
    sensor_msgs::msg::CameraInfo::SharedPtr cinfo_rgb;
    sensor_msgs::msg::CameraInfo::SharedPtr cinfo_depth;
    sensor_msgs::msg::CameraInfo::SharedPtr cinfo_ir;
    TopicStatus st_rgb, st_depth, st_ir;
  };
  std::array<CameraGroup, 3> cams_;  // names + update_rate set in ctor
  int cam_idx_{0};
  QComboBox * cam_sel_{nullptr};
  QLabel * cam_rgb_lbl_{nullptr};
  QLabel * cam_depth_lbl_{nullptr};
  QLabel * cam_ir_lbl_{nullptr};
  QLabel * cam_info_lbl_{nullptr};

  // ── Depth-camera point clouds (3) ──
  struct CloudView {
    QString name;
    QLabel * view_lbl{nullptr};
    QLabel * stats_lbl{nullptr};
    sensor_msgs::msg::PointCloud2::SharedPtr msg;
    TopicStatus status;
  };
  std::array<CloudView, 3> clouds_;

  // ── Laser scans: radar (0) + 4 ultrasonic ──
  struct ScanView {
    QString name;
    QLabel * view_lbl{nullptr};
    QLabel * stats_lbl{nullptr};
    bool polar{true};   // radar: full 360° polar; ultrasonic: narrow fan
    sensor_msgs::msg::LaserScan::SharedPtr msg;
    TopicStatus status;
  };
  std::array<ScanView, 5> scans_;

  // ── IMU ──
  QLabel * imu_lbl_{nullptr};
  sensor_msgs::msg::Imu::SharedPtr imu_msg_;
  TopicStatus imu_status_;

  // UI builders
  void buildUi();
  QWidget * buildCamerasTab();
  QWidget * buildCloudTab();
  QWidget * buildRadarTab();
  QWidget * buildUltrasonicTab();
  QWidget * buildImuTab();

  // Renderers
  QImage renderImage(const sensor_msgs::msg::Image & msg, int max_w = 340);
  QImage renderCloud(const sensor_msgs::msg::PointCloud2 & msg, const QSize & size);
  QImage renderScan(const sensor_msgs::msg::LaserScan & msg, const QSize & size);
  QString cloudStats(const sensor_msgs::msg::PointCloud2 & msg);
  QString scanStats(const sensor_msgs::msg::LaserScan & msg);
  QString cameraInfoText(const CameraGroup & c);
  QString imuText(const sensor_msgs::msg::Imu & msg);

  void refreshAll();  // refresh timer slot
};

}  // namespace ylr1d_hmi

#endif  // YLR1D_HMI__SENSOR_PANEL_HPP_
