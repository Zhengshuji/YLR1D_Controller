#include "ylr1d_hmi/sensor_panel.hpp"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QScrollArea>
#include <QPainter>
#include <QPixmap>
#include <QFont>

#include <algorithm>
#include <cmath>
#include <cstring>

namespace ylr1d_hmi {

namespace {
constexpr double kPi = 3.14159265358979323846;

/// Jet-like color ramp for depth / distance rendering.
inline QRgb depthColor(float t) {
  t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
  const float r = std::min(1.0f, std::max(0.0f, 1.5f - std::abs(4.0f * t - 3.0f)));
  const float g = std::min(1.0f, std::max(0.0f, 1.5f - std::abs(4.0f * t - 2.0f)));
  const float b = std::min(1.0f, std::max(0.0f, 1.5f - std::abs(4.0f * t - 1.0f)));
  return qRgb(static_cast<int>(r * 255.0f),
              static_cast<int>(g * 255.0f),
              static_cast<int>(b * 255.0f));
}
}  // namespace

// ============================================================
// Construction: UI + subscriptions + 10 Hz refresh
// ============================================================
SensorPanel::SensorPanel(rclcpp::Node::SharedPtr node, QWidget * parent)
  : QWidget(parent), node_(node)
{
  // Camera names must be set before buildUi() fills the selector combo.
  static const char * kCamNames[3] = {"Global", "Left", "Right"};
  for (int i = 0; i < 3; ++i) {
    cams_[i].name = kCamNames[i];
    cams_[i].update_rate = 30;
  }

  buildUi();

  // ── Cameras: Global / Left / Right — rgb / depth / ir + their camera_info ──
  static const struct {
    const char * rgb, * depth, * ir;
    const char * rgb_info, * depth_info, * ir_info;
  } kCams[3] = {
    {"/global_camera/rgb/image_raw",        "/global_camera/depth/image_raw",        "/global_camera/infrared/image_raw",
     "/global_camera/rgb/camera_info",      "/global_camera/depth/camera_info",      "/global_camera/infrared/camera_info"},
    {"/left_camera/rgb/image_raw",          "/left_camera/depth/image_raw",          "/left_camera/infrared/image_raw",
     "/left_camera/rgb/camera_info",        "/left_camera/depth/camera_info",        "/left_camera/infrared/camera_info"},
    {"/right_camera/rgb/image_raw",         "/right_camera/depth/image_raw",         "/right_camera/infrared/image_raw",
     "/right_camera/rgb/camera_info",       "/right_camera/depth/camera_info",       "/right_camera/infrared/camera_info"},
  };
  for (int i = 0; i < 3; ++i) {
    subs_.push_back(node_->create_subscription<sensor_msgs::msg::Image>(
      kCams[i].rgb, 1, [this, i](const sensor_msgs::msg::Image::SharedPtr m) {
        cams_[i].rgb = m; cams_[i].st_rgb.touch(); }));
    subs_.push_back(node_->create_subscription<sensor_msgs::msg::Image>(
      kCams[i].depth, 1, [this, i](const sensor_msgs::msg::Image::SharedPtr m) {
        cams_[i].depth = m; cams_[i].st_depth.touch(); }));
    subs_.push_back(node_->create_subscription<sensor_msgs::msg::Image>(
      kCams[i].ir, 1, [this, i](const sensor_msgs::msg::Image::SharedPtr m) {
        cams_[i].ir = m; cams_[i].st_ir.touch(); }));
    subs_.push_back(node_->create_subscription<sensor_msgs::msg::CameraInfo>(
      kCams[i].rgb_info, 1, [this, i](const sensor_msgs::msg::CameraInfo::SharedPtr m) { cams_[i].cinfo_rgb = m; }));
    subs_.push_back(node_->create_subscription<sensor_msgs::msg::CameraInfo>(
      kCams[i].depth_info, 1, [this, i](const sensor_msgs::msg::CameraInfo::SharedPtr m) { cams_[i].cinfo_depth = m; }));
    subs_.push_back(node_->create_subscription<sensor_msgs::msg::CameraInfo>(
      kCams[i].ir_info, 1, [this, i](const sensor_msgs::msg::CameraInfo::SharedPtr m) { cams_[i].cinfo_ir = m; }));
  }

  // ── Depth-camera point clouds (3) ──
  static const struct { const char * name; const char * topic; } kClouds[3] = {
    {"Global", "/global_camera/depth/points"},
    {"Left",   "/left_camera/depth/points"},
    {"Right",  "/right_camera/depth/points"},
  };
  for (int i = 0; i < 3; ++i) {
    auto & cl = clouds_[i];
    cl.name = kClouds[i].name;
    subs_.push_back(node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      kClouds[i].topic, 1, [this, i](const sensor_msgs::msg::PointCloud2::SharedPtr m) {
        clouds_[i].msg = m; clouds_[i].status.touch(); }));
  }

  // ── Radar (polar) + 4 ultrasonic (fan) ──
  scans_[0].name = "Radar";
  subs_.push_back(node_->create_subscription<sensor_msgs::msg::LaserScan>(
    "/radar/scan", 1, [this](const sensor_msgs::msg::LaserScan::SharedPtr m) {
      scans_[0].msg = m; scans_[0].status.touch(); }));

  static const struct { const char * name; const char * topic; } kSonars[4] = {
    {"LF", "/lf_ultrasonic/range"},
    {"RF", "/rf_ultrasonic/range"},
    {"LB", "/lb_ultrasonic/range"},
    {"RB", "/rb_ultrasonic/range"},
  };
  for (int i = 0; i < 4; ++i) {
    auto & s = scans_[i + 1];
    s.name = kSonars[i].name;
    subs_.push_back(node_->create_subscription<sensor_msgs::msg::LaserScan>(
      kSonars[i].topic, 1,
      [this, i](const sensor_msgs::msg::LaserScan::SharedPtr m) {
        scans_[i + 1].msg = m; scans_[i + 1].status.touch(); }));
  }

  // ── IMU ──
  subs_.push_back(node_->create_subscription<sensor_msgs::msg::Imu>(
    "/imu_data", 1, [this](const sensor_msgs::msg::Imu::SharedPtr m) {
      imu_msg_ = m; imu_status_.touch(); }));

  // Refresh at a fixed, low rate (2 Hz) from the latest cached frames —
  // images/clouds/scans are heavy to convert, keep the panel cheap.
  refresh_timer_ = new QTimer(this);
  connect(refresh_timer_, &QTimer::timeout, this, &SensorPanel::refreshAll);
  refresh_timer_->start(500);
}

// ============================================================
// UI
// ============================================================
void SensorPanel::buildUi() {
  auto tabs = new QTabWidget();
  tabs->addTab(buildCamerasTab(), QStringLiteral("Cameras"));
  tabs->addTab(buildCloudTab(), QStringLiteral("Point Cloud"));
  tabs->addTab(buildRadarTab(), QStringLiteral("Radar"));
  tabs->addTab(buildUltrasonicTab(), QStringLiteral("Ultrasonic"));
  tabs->addTab(buildImuTab(), QStringLiteral("IMU"));

  auto lay = new QVBoxLayout(this);
  lay->setContentsMargins(4, 4, 4, 4);
  lay->addWidget(tabs);
}

QWidget * SensorPanel::buildCamerasTab() {
  auto wrap = new QWidget();
  auto lay = new QVBoxLayout(wrap);
  lay->setSpacing(6);

  // Camera selector: switch between Global / Left / Right (render only the
  // selected camera's three streams to keep WSL soft rendering cheap).
  auto sel_row = new QHBoxLayout();
  sel_row->addWidget(new QLabel(QStringLiteral("Camera:")));
  cam_sel_ = new QComboBox();
  for (const auto & c : cams_) cam_sel_->addItem(c.name);
  connect(cam_sel_, qOverload<int>(&QComboBox::currentIndexChanged),
          this, [this](int i) { cam_idx_ = i; });
  sel_row->addWidget(cam_sel_);
  sel_row->addStretch();
  lay->addLayout(sel_row);

  auto gb = new QGroupBox(QStringLiteral("Selected camera"));
  auto v = new QVBoxLayout(gb);

  auto h = new QHBoxLayout();
  cam_rgb_lbl_ = new QLabel(QStringLiteral("RGB …"));
  cam_depth_lbl_ = new QLabel(QStringLiteral("Depth …"));
  cam_ir_lbl_ = new QLabel(QStringLiteral("IR …"));
  for (auto * lbl : {cam_rgb_lbl_, cam_depth_lbl_, cam_ir_lbl_}) {
    lbl->setScaledContents(true);
    lbl->setFixedHeight(180);
    lbl->setAlignment(Qt::AlignCenter);
    lbl->setStyleSheet(QStringLiteral("background:#202020; color:#aaa;"));
  }
  cam_rgb_lbl_->setFixedWidth(320);
  cam_depth_lbl_->setFixedWidth(240);
  cam_ir_lbl_->setFixedWidth(240);
  h->addWidget(cam_rgb_lbl_);
  h->addWidget(cam_depth_lbl_);
  h->addWidget(cam_ir_lbl_);
  h->addStretch();
  v->addLayout(h);

  cam_info_lbl_ = new QLabel(QStringLiteral("camera info …"));
  cam_info_lbl_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  v->addWidget(cam_info_lbl_);

  lay->addWidget(gb);
  lay->addStretch();

  auto scroll = new QScrollArea();
  scroll->setWidget(wrap);
  scroll->setWidgetResizable(true);
  return scroll;
}

QWidget * SensorPanel::buildCloudTab() {
  auto w = new QWidget();
  auto lay = new QHBoxLayout(w);
  for (int i = 0; i < 3; ++i) {
    auto & cl = clouds_[i];
    auto box = new QVBoxLayout();
    auto title = new QLabel(cl.name);
    title->setAlignment(Qt::AlignCenter);
    cl.view_lbl = new QLabel();
    cl.view_lbl->setFixedSize(360, 280);
    cl.view_lbl->setStyleSheet(QStringLiteral("background:#000;"));
    cl.stats_lbl = new QLabel(cl.name + QStringLiteral(": waiting"));
    box->addWidget(title);
    box->addWidget(cl.view_lbl, 0, Qt::AlignCenter);
    box->addWidget(cl.stats_lbl);
    auto holder = new QWidget();
    holder->setLayout(box);
    lay->addWidget(holder);
  }
  lay->addStretch();
  return w;
}

QWidget * SensorPanel::buildRadarTab() {
  auto w = new QWidget();
  auto v = new QVBoxLayout(w);
  auto & s = scans_[0];
  s.view_lbl = new QLabel();
  s.view_lbl->setFixedSize(640, 480);
  s.view_lbl->setStyleSheet(QStringLiteral("background:#000;"));
  s.stats_lbl = new QLabel(QStringLiteral("Radar: waiting"));
  v->addWidget(s.view_lbl, 0, Qt::AlignCenter);
  v->addWidget(s.stats_lbl, 0, Qt::AlignCenter);
  v->addStretch();
  return w;
}

QWidget * SensorPanel::buildUltrasonicTab() {
  auto w = new QWidget();
  auto lay = new QHBoxLayout(w);
  for (int i = 1; i < 5; ++i) {
    auto & s = scans_[i];
    auto box = new QVBoxLayout();
    auto title = new QLabel(s.name);
    title->setAlignment(Qt::AlignCenter);
    s.view_lbl = new QLabel();
    s.view_lbl->setFixedSize(180, 180);
    s.view_lbl->setStyleSheet(QStringLiteral("background:#000;"));
    s.stats_lbl = new QLabel(s.name + QStringLiteral(": waiting"));
    s.stats_lbl->setWordWrap(true);
    box->addWidget(title);
    box->addWidget(s.view_lbl, 0, Qt::AlignCenter);
    box->addWidget(s.stats_lbl);
    auto holder = new QWidget();
    holder->setLayout(box);
    lay->addWidget(holder);
  }
  lay->addStretch();
  return w;
}

QWidget * SensorPanel::buildImuTab() {
  auto w = new QWidget();
  auto v = new QVBoxLayout(w);
  imu_lbl_ = new QLabel(QStringLiteral("IMU: waiting"));
  imu_lbl_->setFont(QFont(QStringLiteral("Monospace")));
  imu_lbl_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  v->addWidget(imu_lbl_);
  v->addStretch();
  return w;
}

// ============================================================
// Renderers
// ============================================================
QImage SensorPanel::renderImage(const sensor_msgs::msg::Image & msg, int max_w) {
  const int w = static_cast<int>(msg.width);
  const int h = static_cast<int>(msg.height);
  if (w <= 0 || h <= 0 || msg.data.empty()) return {};
  const std::string & enc = msg.encoding;

  QImage out;
  if (enc == "rgb8" || enc == "RGB8") {
    out = QImage(msg.data.data(), w, h, msg.step, QImage::Format_RGB888).copy();
  } else if (enc == "bgr8" || enc == "BGR8") {
    out = QImage(msg.data.data(), w, h, msg.step, QImage::Format_BGR888).copy();
  } else if (enc == "32FC1") {  // depth in meters
    out = QImage(w, h, QImage::Format_RGB888);
    const float * p = reinterpret_cast<const float *>(msg.data.data());
    constexpr float kDmax = 4.0f;
    for (int y = 0; y < h; ++y) {
      uchar * row = out.scanLine(y);
      const float * prow = p + static_cast<size_t>(y) * w;
      for (int x = 0; x < w; ++x) {
        const float v = prow[x];
        const float t = (std::isnan(v) || v <= 0.0f) ? 0.0f : std::min(v / kDmax, 1.0f);
        const QRgb c = depthColor(t);
        row[x * 3] = qRed(c);
        row[x * 3 + 1] = qGreen(c);
        row[x * 3 + 2] = qBlue(c);
      }
    }
  } else if (enc == "16UC1") {  // depth in millimeters
    out = QImage(w, h, QImage::Format_RGB888);
    const uint16_t * p = reinterpret_cast<const uint16_t *>(msg.data.data());
    constexpr float kDmax = 4000.0f;
    for (int y = 0; y < h; ++y) {
      uchar * row = out.scanLine(y);
      const uint16_t * prow = p + static_cast<size_t>(y) * w;
      for (int x = 0; x < w; ++x) {
        const float t = std::min(static_cast<float>(prow[x]) / kDmax, 1.0f);
        const QRgb c = depthColor(t);
        row[x * 3] = qRed(c);
        row[x * 3 + 1] = qGreen(c);
        row[x * 3 + 2] = qBlue(c);
      }
    }
  } else if (enc == "8UC1" || enc == "mono8") {  // grayscale
    out = QImage(w, h, QImage::Format_Grayscale8);
    for (int y = 0; y < h; ++y)
      std::memcpy(out.scanLine(y), msg.data.data() + static_cast<size_t>(y) * msg.step, w);
  } else {  // fallback: treat as RGB888
    out = QImage(msg.data.data(), w, h, msg.step, QImage::Format_RGB888).copy();
  }

  if (out.width() > max_w)
    out = out.scaled(max_w, max_w * h / w, Qt::KeepAspectRatio, Qt::SmoothTransformation);
  return out;
}

QImage SensorPanel::renderCloud(const sensor_msgs::msg::PointCloud2 & msg, const QSize & size) {
  QImage img(size, QImage::Format_RGB888);
  img.fill(Qt::black);
  if (msg.data.empty() || msg.point_step == 0) return img;

  const size_t pstep = msg.point_step;
  const size_t npts = msg.data.size() / pstep;
  const uint8_t * data = msg.data.data();

  size_t ox = 0, oz = 8;
  for (const auto & f : msg.fields) {
    if (f.name == "x") ox = f.offset;
    else if (f.name == "z") oz = f.offset;
  }

  const size_t sample = std::max<size_t>(1, npts / 20000);  // ≤ 20k drawn points

  // Bounds over x (horizontal) and z (forward depth), y discarded (top view).
  float xmin = 0.0f, xmax = 0.0f, zmin = 0.0f, zmax = 0.0f;
  bool first = true;
  for (size_t i = 0; i < npts; i += sample) {
    const uint8_t * p = data + i * pstep;
    float x, z;
    std::memcpy(&x, p + ox, 4);
    std::memcpy(&z, p + oz, 4);
    if (!std::isfinite(x) || !std::isfinite(z)) continue;
    if (first) { xmin = xmax = x; zmin = zmax = z; first = false; }
    else {
      xmin = std::min(xmin, x); xmax = std::max(xmax, x);
      zmin = std::min(zmin, z); zmax = std::max(zmax, z);
    }
  }
  if (first) return img;

  float dx = xmax - xmin;
  float dz = zmax - zmin;
  if (dx < 1e-4f) dx = 1.0f;
  if (dz < 1e-4f) dz = 1.0f;
  xmin -= 0.05f * dx; xmax += 0.05f * dx;
  zmin -= 0.05f * dz; zmax += 0.05f * dz;
  dx = xmax - xmin; dz = zmax - zmin;

  const int W = size.width(), H = size.height();
  for (size_t i = 0; i < npts; i += sample) {
    const uint8_t * p = data + i * pstep;
    float x, z;
    std::memcpy(&x, p + ox, 4);
    std::memcpy(&z, p + oz, 4);
    if (!std::isfinite(x) || !std::isfinite(z)) continue;
    const int px = static_cast<int>((x - xmin) / dx * (W - 1));
    const int py = static_cast<int>((z - zmin) / dz * (H - 1));
    if (px < 0 || px >= W || py < 0 || py >= H) continue;
    const QRgb c = depthColor(std::min((z - zmin) / dz, 1.0f));
    uchar * row = img.scanLine(py);
    row[px * 3] = qRed(c);
    row[px * 3 + 1] = qGreen(c);
    row[px * 3 + 2] = qBlue(c);
  }
  return img;
}

QImage SensorPanel::renderScan(const sensor_msgs::msg::LaserScan & msg, const QSize & size) {
  QImage img(size, QImage::Format_RGB888);
  img.fill(Qt::black);
  const int W = size.width(), H = size.height();
  const int cx = W / 2, cy = H / 2;
  const int R = std::min(W, H) / 2 - 4;

  // Autoscale radius to the largest valid measurement for a clearer picture.
  float rmax = msg.range_max;
  for (float r : msg.ranges)
    if (std::isfinite(r) && r > rmax) rmax = r;
  if (rmax <= msg.range_min) rmax = msg.range_max;

  QPainter p(&img);
  p.setPen(QPen(QColor(120, 200, 255)));
  for (size_t i = 0; i < msg.ranges.size(); ++i) {
    const float r = msg.ranges[i];
    if (!std::isfinite(r) || r < msg.range_min) continue;
    const float a = msg.angle_min + static_cast<float>(i) * msg.angle_increment;
    const float rr = std::min(r / rmax, 1.0f) * R;
    p.drawPoint(cx + static_cast<int>(rr * std::cos(a)),
                cy + static_cast<int>(rr * std::sin(a)));
  }
  // Sensor center marker
  p.setPen(QPen(QColor(0, 200, 0)));
  p.drawLine(cx - 6, cy, cx + 6, cy);
  p.drawLine(cx, cy - 6, cx, cy + 6);
  p.end();
  return img;
}

QString SensorPanel::cloudStats(const sensor_msgs::msg::PointCloud2 & msg) {
  if (msg.data.empty() || msg.point_step == 0) return QStringLiteral("no data");
  const size_t pstep = msg.point_step;
  const size_t npts = msg.data.size() / pstep;
  size_t ox = 0, oy = 4, oz = 8;
  for (const auto & f : msg.fields) {
    if (f.name == "x") ox = f.offset;
    else if (f.name == "y") oy = f.offset;
    else if (f.name == "z") oz = f.offset;
  }
  const uint8_t * data = msg.data.data();
  float dmin = std::numeric_limits<float>::max(), dmax = 0.0f;
  size_t nvalid = 0;
  const size_t sample = std::max<size_t>(1, npts / 20000);
  for (size_t i = 0; i < npts; i += sample) {
    const uint8_t * p = data + i * pstep;
    float x, y, z;
    std::memcpy(&x, p + ox, 4);
    std::memcpy(&y, p + oy, 4);
    std::memcpy(&z, p + oz, 4);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
    const float d = std::sqrt(x * x + y * y + z * z);
    dmin = std::min(dmin, d);
    dmax = std::max(dmax, d);
    ++nvalid;
  }
  if (nvalid == 0) return QStringLiteral("no valid points");
  return QStringLiteral("%1 pts · min %2 m · max %3 m")
    .arg(npts).arg(dmin, 0, 'f', 2).arg(dmax, 0, 'f', 2);
}

QString SensorPanel::scanStats(const sensor_msgs::msg::LaserScan & msg) {
  int nvalid = 0;
  float rmin = msg.range_max, rmax = 0.0f, rsum = 0.0f;
  for (float r : msg.ranges) {
    if (!std::isfinite(r)) continue;
    ++nvalid;
    rmin = std::min(rmin, r);
    rmax = std::max(rmax, r);
    rsum += r;
  }
  if (nvalid == 0) return QStringLiteral("no valid readings");
  return QStringLiteral("%1 samples · %2 valid · min %3 m · max %4 m · avg %5 m")
    .arg(msg.ranges.size()).arg(nvalid)
    .arg(rmin, 0, 'f', 2).arg(rmax, 0, 'f', 2)
    .arg(rsum / nvalid, 0, 'f', 2);
}

QString SensorPanel::cameraInfoText(const CameraGroup & c) {
  auto fmt = [](const sensor_msgs::msg::CameraInfo::SharedPtr & ci) -> QString {
    if (!ci) return QStringLiteral("—");
    const double fx = ci->k[0];
    const double hfov = fx > 0.0
      ? 2.0 * std::atan2(static_cast<double>(ci->width), 2.0 * fx) * 180.0 / kPi
      : 0.0;
    return QStringLiteral("%1×%2 · hfov %3°").arg(ci->width).arg(ci->height)
      .arg(hfov, 0, 'f', 1);
  };
  return QStringLiteral("<b>%1</b> · update %2 Hz<br>"
                        "RGB&nbsp;&nbsp;: %3<br>Depth: %4<br>IR&nbsp;&nbsp;&nbsp;: %5<br>"
                        "<span style='color:#7ac;'>RGB&nbsp;&nbsp;: %6<br>"
                        "Depth: %7<br>IR&nbsp;&nbsp;&nbsp;: %8</span>")
    .arg(c.name).arg(c.update_rate)
    .arg(fmt(c.cinfo_rgb), fmt(c.cinfo_depth), fmt(c.cinfo_ir))
    .arg(c.st_rgb.text(), c.st_depth.text(), c.st_ir.text());
}

QString SensorPanel::imuText(const sensor_msgs::msg::Imu & msg) {
  const auto & o = msg.orientation;
  // Quaternion → Euler (approx, ZYX)
  const double sinr = 2.0 * (o.w * o.x + o.y * o.z);
  const double cosr = 1.0 - 2.0 * (o.x * o.x + o.y * o.y);
  const double roll = std::atan2(sinr, cosr);
  const double sinp = 2.0 * (o.w * o.y - o.z * o.x);
  const double pitch = std::abs(sinp) >= 1.0 ? std::copysign(kPi / 2.0, sinp)
                                             : std::asin(sinp);
  const double siny = 2.0 * (o.w * o.z + o.x * o.y);
  const double cosy = 1.0 - 2.0 * (o.y * o.y + o.z * o.z);
  const double yaw = std::atan2(siny, cosy);
  const auto r2d = [](double r) { return r * 180.0 / kPi; };

  return QStringLiteral("orientation   : roll %1°   pitch %2°   yaw %3°\n"
                        "angular vel   : x %4   y %5   z %6   (rad/s)\n"
                        "linear accel  : x %7   y %8   z %9   (m/s²)")
    .arg(r2d(roll), 0, 'f', 1).arg(r2d(pitch), 0, 'f', 1).arg(r2d(yaw), 0, 'f', 1)
    .arg(msg.angular_velocity.x, 0, 'f', 3)
    .arg(msg.angular_velocity.y, 0, 'f', 3)
    .arg(msg.angular_velocity.z, 0, 'f', 3)
    .arg(msg.linear_acceleration.x, 0, 'f', 3)
    .arg(msg.linear_acceleration.y, 0, 'f', 3)
    .arg(msg.linear_acceleration.z, 0, 'f', 3);
}

// ============================================================
// Refresh
// ============================================================
void SensorPanel::refreshAll() {
  auto & c = cams_[cam_idx_];
  if (c.rgb && cam_rgb_lbl_)
    cam_rgb_lbl_->setPixmap(QPixmap::fromImage(renderImage(*c.rgb)));
  if (c.depth && cam_depth_lbl_)
    cam_depth_lbl_->setPixmap(QPixmap::fromImage(renderImage(*c.depth)));
  if (c.ir && cam_ir_lbl_)
    cam_ir_lbl_->setPixmap(QPixmap::fromImage(renderImage(*c.ir)));
  if (cam_info_lbl_)
    cam_info_lbl_->setText(cameraInfoText(c));
  for (auto & cl : clouds_) {
    if (cl.msg && cl.view_lbl)
      cl.view_lbl->setPixmap(QPixmap::fromImage(renderCloud(*cl.msg, cl.view_lbl->size())));
    if (cl.stats_lbl) {
      QString t = cl.name + QStringLiteral("  ") + cl.status.text();
      if (cl.msg) t += QStringLiteral("\n") + cloudStats(*cl.msg);
      cl.stats_lbl->setText(t);
    }
  }
  for (auto & s : scans_) {
    if (s.msg && s.view_lbl)
      s.view_lbl->setPixmap(QPixmap::fromImage(renderScan(*s.msg, s.view_lbl->size())));
    if (s.stats_lbl) {
      QString t = s.name + QStringLiteral("  ") + s.status.text();
      if (s.msg) t += QStringLiteral("\n") + scanStats(*s.msg);
      s.stats_lbl->setText(t);
    }
  }
  if (imu_lbl_) {
    QString t = QStringLiteral("IMU  ") + imu_status_.text();
    if (imu_msg_) t += QStringLiteral("\n") + imuText(*imu_msg_);
    imu_lbl_->setText(t);
  }
}

}  // namespace ylr1d_hmi
