#ifndef PLANT__PLANT_HPP_
#define PLANT__PLANT_HPP_

/// 被控对象汇总头：消费方只需 include 本文件即可获得全部被控对象类型。
/// 新增被控对象时，在 plant/<name>/ 下建对象，并在此追加一行 include；
/// 消费方代码无需改动。
#include "plant/position/position_plant.hpp"
#include "plant/position_group/position_group_plant.hpp"
#include "plant/velocity/velocity_plant.hpp"
#include "plant/velocity_group/velocity_group_plant.hpp"

#endif  // PLANT__PLANT_HPP_
