#ifndef YLR1D_HMI__PANELS__MONITOR_NODES_HPP_
#define YLR1D_HMI__PANELS__MONITOR_NODES_HPP_

#include <QString>

namespace ylr1d_hmi {

/// One expected node the monitor tracks for liveness.
/// `optional` nodes (e.g. translate_server, rviz2) that are missing show a
/// yellow "missing (optional)" row but do NOT raise a red anomaly — they only
/// exist in some launch configurations.
struct ExpectedNode {
  QString name;  // node name (namespace slash stripped)
  QString role;
  bool optional{false};
};

/// Nodes the monitor expects for the standard position-sim setup.
inline const ExpectedNode kExpectedNodes[] = {
  {"robot_state_publisher", "URDF + TF", false},
  {"gazebo_ros2_control",   "ros2_control plugin", false},
  {"controller_manager",    "controller manager", false},
  {"arm_simulate",          "arm soft-sim (position)", false},
  {"chassis_simulate",      "chassis soft-sim (position)", false},
  {"translate_server",      "translate (3 action servers)", true},
  {"rviz2",                 "3D view", true},
};

/// Compare a graph node name against an expected short name, tolerating
/// leading slashes and namespaces (take the last path segment).
inline bool nodeNameMatch(const QString & detected, const QString & expected) {
  QString d = detected;
  while (d.startsWith(QLatin1Char('/'))) d = d.mid(1);
  const int slash = d.lastIndexOf(QLatin1Char('/'));
  if (slash >= 0) d = d.mid(slash + 1);
  return d == expected;
}

}  // namespace ylr1d_hmi

#endif  // YLR1D_HMI__PANELS__MONITOR_NODES_HPP_
