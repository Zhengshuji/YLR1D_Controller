#ifndef CONTROLLER__CONTROLLER_HPP_
#define CONTROLLER__CONTROLLER_HPP_

/// 控制器汇总头：消费方只需 include 本文件即可获得全部控制器类型。
/// 新增控制器时，在 controller/<name>/ 下建对象，并在此追加一行 include；
/// 消费方代码无需改动（配合模板化关节控制器使用）。
#include "controller/pid/pid.hpp"
#include "controller/proportional/proportional_controller.hpp"

#endif  // CONTROLLER__CONTROLLER_HPP_
