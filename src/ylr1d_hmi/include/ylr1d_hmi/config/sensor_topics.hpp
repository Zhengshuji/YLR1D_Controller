#ifndef YLR1D_HMI__CONFIG__SENSOR_TOPICS_HPP_
#define YLR1D_HMI__CONFIG__SENSOR_TOPICS_HPP_

namespace ylr1d_hmi {

/// One robot camera unit: RGB / depth / IR streams plus their camera_info.
/// Names and topics match ylr1d_description (the SDF world model).
struct CameraSpec {
  const char * name;
  const char * rgb, * depth, * ir;
  const char * rgb_info, * depth_info, * ir_info;
  int update_rate;
};

inline constexpr CameraSpec kCameras[3] = {
  {"Global",
   "/global_camera/rgb/image_raw",        "/global_camera/depth/image_raw",        "/global_camera/infrared/image_raw",
   "/global_camera/rgb/camera_info",      "/global_camera/depth/camera_info",      "/global_camera/infrared/camera_info",
   30},
  {"Left",
   "/left_camera/rgb/image_raw",          "/left_camera/depth/image_raw",          "/left_camera/infrared/image_raw",
   "/left_camera/rgb/camera_info",        "/left_camera/depth/camera_info",        "/left_camera/infrared/camera_info",
   30},
  {"Right",
   "/right_camera/rgb/image_raw",         "/right_camera/depth/image_raw",         "/right_camera/infrared/image_raw",
   "/right_camera/rgb/camera_info",       "/right_camera/depth/camera_info",       "/right_camera/infrared/camera_info",
   30},
};

/// Depth-camera point clouds — one per camera.
inline constexpr const char * kCloudTopics[3] = {
  "/global_camera/depth/points",
  "/left_camera/depth/points",
  "/right_camera/depth/points",
};

inline constexpr const char * kRadarTopic = "/radar/scan";

/// Short display names for the four ultrasonic units (LF / RF / LB / RB).
inline constexpr const char * kSonarNames[4] = {"LF", "RF", "LB", "RB"};

inline constexpr const char * kSonarTopics[4] = {
  "/lf_ultrasonic/range",
  "/rf_ultrasonic/range",
  "/lb_ultrasonic/range",
  "/rb_ultrasonic/range",
};

inline constexpr const char * kImuTopic = "/imu_data";

/// Flat liveness table used by the monitor's Sensors tab. Topic strings point
/// at the atomic tables above so there is exactly one source of truth.
struct SensorSpec {
  const char * label;
  const char * topic;
  int kind;  // 0=Image, 1=PointCloud2, 2=LaserScan, 3=Imu
};

inline const SensorSpec kSensorSpecs[] = {
  {"Global Cam RGB",   kCameras[0].rgb,   0},
  {"Global Cam Depth", kCameras[0].depth, 0},
  {"Global Cam IR",    kCameras[0].ir,    0},
  {"Left Cam RGB",     kCameras[1].rgb,   0},
  {"Left Cam Depth",   kCameras[1].depth, 0},
  {"Left Cam IR",      kCameras[1].ir,    0},
  {"Right Cam RGB",    kCameras[2].rgb,   0},
  {"Right Cam Depth",  kCameras[2].depth, 0},
  {"Right Cam IR",     kCameras[2].ir,    0},
  {"Global Cloud",     kCloudTopics[0],   1},
  {"Left Cloud",       kCloudTopics[1],   1},
  {"Right Cloud",      kCloudTopics[2],   1},
  {"Radar",            kRadarTopic,       2},
  {"Ultrasonic LF",    kSonarTopics[0],   2},
  {"Ultrasonic RF",    kSonarTopics[1],   2},
  {"Ultrasonic LB",    kSonarTopics[2],   2},
  {"Ultrasonic RB",    kSonarTopics[3],   2},
  {"IMU",              kImuTopic,         3},
};

}  // namespace ylr1d_hmi

#endif  // YLR1D_HMI__CONFIG__SENSOR_TOPICS_HPP_
