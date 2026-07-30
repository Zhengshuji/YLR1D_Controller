#include "ylr1d_hmi/hmi_window.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QSplitter>
#include <QHeaderView>
#include <QTreeWidgetItem>

namespace ylr1d_hmi {

// ============================================================
// Joint group definitions
// ============================================================
struct JointDef {
  std::string name;
  QString label;
  bool is_velocity{false};
  double lower{-3.14};
  double upper{3.14};
};

static std::vector<JointDef> chassis_joints = {
  {"Joint_Base_to_RFWheelF",    "RF_Steer",  false, -3.14, 3.14},
  {"Joint_Base_to_LFWheelF",    "LF_Steer",  false, -3.14, 3.14},
  {"Joint_Base_to_RBWheelF",    "RB_Steer",  false, -3.14, 3.14},
  {"Joint_Base_to_LBWheelF",    "LB_Steer",  false, -3.14, 3.14},
  {"Joint_RFWheelF_to_RFWheel", "RF_Wheel",  true},
  {"Joint_LFWheelF_to_LFWheel", "LF_Wheel",  true},
  {"Joint_RBWheelF_to_RBWheel", "RB_Wheel",  true},
  {"Joint_LBWheelF_to_LBWheel", "LB_Wheel",  true},
};

static std::vector<JointDef> torso_joints = {
  {"Joint_Base_to_Body1",    "Lift",    false, -0.30,  0.30},
  {"Joint_Body1_to_Body2",   "Yaw",     false, -3.14,  3.14},
  {"Joint_Body2_to_Body3",   "Pitch1",  false, -1.57,  1.57},
  {"Joint_Body3_to_Body4",   "Pitch2",  false, -1.57,  1.57},
};

static std::vector<JointDef> left_arm_joints = {
  {"Joint_Body2_to_LeftArm1",    "Shoulder1", false, -2.62,  2.62},
  {"Joint_LeftArm1_to_LeftArm2", "Shoulder2", false, -1.57,  1.83},
  {"Joint_LeftArm2_to_LeftArm3", "Shoulder3", false, -2.62,  2.62},
  {"Joint_LeftArm3_to_LeftArm4", "Elbow1",    false, -1.57,  1.57},
  {"Joint_LeftArm4_to_LeftArm5", "Elbow2",    false, -2.62,  2.62},
  {"Joint_LeftArm5_to_LeftArm6", "Wrist1",    false, -2.09,  2.09},
  {"Joint_LeftArm6_to_LeftArm7", "Wrist2",    false, -6.28,  6.28},
  {"Joint_LeftArm7_to_LeftFinger1","Finger1", false, -0.014, 0.0},
  {"Joint_LeftArm7_to_LeftFinger2","Finger2", false, -0.014, 0.0},
};

static std::vector<JointDef> right_arm_joints = {
  {"Joint_Body2_RightArm1",      "Shoulder1", false, -2.62,  2.62},
  {"Joint_RightArm1_to_RightArm2","Shoulder2", false, -1.57,  1.83},
  {"Joint_RightArm2_to_RightArm3","Shoulder3", false, -2.62,  2.62},
  {"Joint_RightArm3_to_RightArm4","Elbow1",    false, -1.57,  1.57},
  {"Joint_RightArm4_to_RightArm5","Elbow2",    false, -2.62,  2.62},
  {"Joint_RightArm5_to_RightArm6","Wrist1",    false, -2.09,  2.09},
  {"Joint_RightArm6_to_RightArm7","Wrist2",    false, -6.28,  6.28},
  {"Joint_RightArm7_to_RightFinger1","Finger1", false, 0.0, 0.014},
  {"Joint_RightArm7_to_RightFinger2","Finger2", false, 0.0, 0.014},
};

// ============================================================
// HmiWindow
// ============================================================

HmiWindow::HmiWindow(rclcpp::Node::SharedPtr node)
  : node_(node)
{
  // ROS2 subscribe / publish
  js_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
      for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
        auto it = name_to_idx_.find(msg->name[i]);
        if (it != name_to_idx_.end()) {
          auto & j = joints_[it->second];
          j.position = std::isnan(msg->position[i]) ? j.position : msg->position[i];
          if (i < msg->velocity.size())
            j.velocity = std::isnan(msg->velocity[i]) ? 0.0 : msg->velocity[i];
        }
      }
    });

  desired_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/desired_joint_states", 10);

  // NOTE: buildUiLite() is NOT called here — the caller (main or subclass)
  // decides which layout to use.

  // ROS spin timer (50 Hz)
  ros_timer_ = new QTimer(this);
  connect(ros_timer_, &QTimer::timeout, this, &HmiWindow::onRosSpin);
  ros_timer_->start(20);
}

HmiWindow::~HmiWindow() {
  ros_timer_->stop();
}

// ============================================================
// Lite layout: Observer top + Controller bottom
// ============================================================
void HmiWindow::buildUiLite() {
  setWindowTitle("YLR1D HMI (lite)");
  resize(1200, 800);

  auto central = new QWidget(this);
  setCentralWidget(central);
  auto main_layout = new QVBoxLayout(central);
  main_layout->setContentsMargins(4, 4, 4, 4);
  main_layout->setSpacing(4);

  // Vertical split: top=Observer, bottom=Controller
  auto splitter = new QSplitter(Qt::Vertical);

  // Observer
  auto obs = buildObserver(nullptr);
  splitter->addWidget(obs);

  // Controller (scrollable)
  auto ctrl_scroll = new QScrollArea();
  ctrl_scroll->setWidgetResizable(true);
  ctrl_scroll->setWidget(buildController(nullptr));
  splitter->addWidget(ctrl_scroll);

  splitter->setStretchFactor(0, 1);
  splitter->setStretchFactor(1, 1);

  main_layout->addWidget(splitter);
}

// ============================================================
// Observer: QTreeWidget
// ============================================================
QWidget * HmiWindow::buildObserver(QWidget * parent) {
  auto w = parent ? new QWidget(parent) : new QWidget();
  auto lay = new QVBoxLayout(w);
  lay->setContentsMargins(2, 2, 2, 2);

  auto title = new QLabel("<b>Joint Observer</b>");
  lay->addWidget(title);

  observer_tree_ = new QTreeWidget();
  observer_tree_->setColumnCount(3);
  observer_tree_->setHeaderLabels({"Joint", "Position (rad)", "Velocity (rad/s)"});
  observer_tree_->header()->setStretchLastSection(true);
  observer_tree_->header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  observer_tree_->setRootIsDecorated(true);
  observer_tree_->setAlternatingRowColors(true);
  observer_tree_->setAnimated(true);

  auto add_group = [&](const std::string & group_name,
                        const std::vector<JointDef> & defs) {
    auto root = new QTreeWidgetItem(observer_tree_, {QString::fromStdString(group_name)});
    root->setExpanded(true);
    QFont bold = root->font(0);
    bold.setBold(true);
    root->setFont(0, bold);
    for (auto & d : defs) {
      auto item = new QTreeWidgetItem(root, {d.label, "—", "—"});
      item->setData(0, Qt::UserRole, QString::fromStdString(d.name));
    }
  };

  add_group("Chassis (8)", chassis_joints);
  add_group("Torso (4)", torso_joints);
  add_group("Left Arm (9)", left_arm_joints);
  add_group("Right Arm (9)", right_arm_joints);

  lay->addWidget(observer_tree_);
  return w;
}

// ============================================================
// Controller: 4 groups side by side
// ============================================================
QWidget * HmiWindow::buildController(QWidget * parent) {
  auto w = parent ? new QWidget(parent) : new QWidget();
  auto outer_lay = new QVBoxLayout(w);
  outer_lay->setContentsMargins(2, 2, 2, 2);

  auto title = new QLabel("<b>Joint Controller</b>");
  outer_lay->addWidget(title);

  auto hlay = new QHBoxLayout();
  outer_lay->addLayout(hlay);

  auto add_group = [&](const QString & group_name,
                        const std::vector<JointDef> & defs,
                        QBoxLayout * parent_layout) {
    auto gb = new QGroupBox(group_name);
    auto glay = new QVBoxLayout(gb);
    glay->setSpacing(2);

    for (auto & d : defs) {
      // Row: [label + slider + spinbox]
      auto row = new QHBoxLayout();
      row->setSpacing(3);

      auto lbl = new QLabel(d.label);
      lbl->setFixedWidth(55);
      row->addWidget(lbl);

      auto slider = new QSlider(Qt::Horizontal);
      slider->setValue(0);
      row->addWidget(slider, 1);

      auto spin = new QDoubleSpinBox();
      spin->setDecimals(3);
      spin->setValue(0.0);
      spin->setFixedWidth(80);
      row->addWidget(spin);

      if (d.is_velocity) {
        slider->setRange(-500, 500);
        spin->setRange(-5.0, 5.0);
        spin->setSingleStep(0.1);
      } else {
        slider->setRange(
          static_cast<int>(d.lower * 100),
          static_cast<int>(d.upper * 100));
        spin->setRange(d.lower, d.upper);
        spin->setSingleStep(0.05);
      }

      glay->addLayout(row);

      JointInfo ji;
      ji.name = d.name;
      ji.label = d.label;
      ji.is_velocity = d.is_velocity;
      ji.slider = slider;
      ji.spin = spin;

      size_t idx = joints_.size();
      joints_.push_back(ji);
      name_to_idx_[d.name] = idx;

      connect(slider, &QSlider::valueChanged, this, [this, idx](int v) {
        if (!joints_[idx].spin->hasFocus())
          joints_[idx].spin->setValue(static_cast<double>(v) / 100.0);
        onSliderChanged(v);
      });
      connect(spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
              this, [this, idx](double v) { onSpinChanged(v); });
    }
    parent_layout->addWidget(gb);
  };

  add_group("Chassis", chassis_joints, hlay);
  add_group("Torso", torso_joints, hlay);
  add_group("Left Arm", left_arm_joints, hlay);
  add_group("Right Arm", right_arm_joints, hlay);

  return w;
}

// ============================================================
// Slots
// ============================================================
void HmiWindow::onRosSpin() {
  rclcpp::spin_some(node_);

  if (!observer_tree_) return;
  for (int gi = 0; gi < observer_tree_->topLevelItemCount(); ++gi) {
    auto * root = observer_tree_->topLevelItem(gi);
    for (int ci = 0; ci < root->childCount(); ++ci) {
      auto * item = root->child(ci);
      QString name = item->data(0, Qt::UserRole).toString();
      auto it = name_to_idx_.find(name.toStdString());
      if (it != name_to_idx_.end()) {
        auto & j = joints_[it->second];
        item->setText(1, QString::number(j.position, 'f', 4));
        item->setText(2, QString::number(j.velocity, 'f', 4));
      }
    }
  }
}

void HmiWindow::onSliderChanged(int value) {
  for (auto & j : joints_) {
    if (j.slider && j.slider->value() == value && j.slider->hasFocus()) {
      j.desired = static_cast<double>(value) / 100.0;
      break;
    }
  }
  publishDesired();
}

void HmiWindow::onSpinChanged(double value) {
  for (auto & j : joints_) {
    if (j.spin && j.spin->hasFocus()) {
      j.desired = value;
      int sv = static_cast<int>(value * 100);
      j.slider->setValue(sv);
      break;
    }
  }
  publishDesired();
}

void HmiWindow::publishDesired() {
  auto msg = sensor_msgs::msg::JointState();
  msg.header.stamp = node_->now();

  for (size_t i = 0; i < joints_.size(); ++i) {
    msg.name.push_back(joints_[i].name);
    if (joints_[i].is_velocity) {
      msg.position.push_back(0.0);
      msg.velocity.push_back(joints_[i].desired);
    } else {
      msg.position.push_back(joints_[i].desired);
      msg.velocity.push_back(0.0);
    }
  }
  desired_pub_->publish(msg);
}

}  // namespace ylr1d_hmi
