#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <gflags/gflags.h>
#include <fstream>
#include <jsoncpp/json/json.h>
#include <iostream>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace examples {
namespace collaboration_station {

class HighLevelPlanner {
 public:

  HighLevelPlanner(Vector6<double> iiwa_pose,
                  std::string pose_file,
                  unsigned int kuka_num,
                  int max_num_objects);
  bool HasMorePlan();
  void MoveToNewPlan();
  Vector6<double> NewObject();
  math::RigidTransformd NextIiwaMovement();
  double NextWsgMovement();
  bool NextObject();
  void Update(Vector6<double> object_state);

  int AddOne();

  std::string GetChosenPlan();
  std::string GetCurrentTask();
  double GetObjectGraspHeight();
  std::vector<std::string> GetObjectList();
  std::string GetObjectName();
  int GetObjectNum();
  int GetCurrentTaskNum();
  double Print();

  Json::Value JsonParser(std::string file_name);

 private:
  unsigned int randnum_;
  unsigned int max_num_objects_;
  unsigned int max_task_num_;
  unsigned int object_num_;
  unsigned int task_num_;
  unsigned int kuka_num_;
  int wsg_grasp_depth_ = 0.1;
  double current_wsg_state_;
  double grasp_height_;
  Json::Value pose_list_;
  Json::Value object_list_;
  math::RigidTransformd current_iiwa_state_;
  std::string chosen_plan_;
  std::string current_task_;
  std::string object_name_;
  Vector6<double> cabinet_row_pose_;
  Vector6<double> iiwa_pose_;
  Vector6<double> object_state_;
  Vector6<double> pick_pose_;
  Vector6<double> place_pose_;

};

}  // namespace collaboration_station
}  // namespace examples
}  // namespace drake
