#include "drake/examples/collaboration_station/kuka/high_level_planner.h"

namespace drake {
namespace examples {
namespace collaboration_station {


HighLevelPlanner::HighLevelPlanner(Vector6<double> iiwa_pose,
                                   std::string pose_file,
                                   unsigned int kuka_num){
  pose_list_ = 
    JsonParser(pose_file);
  object_list_ = 
    JsonParser("drake/examples/collaboration_station/object_list.json");

  current_wsg_state_ = 100; //Default
  object_num_ = 0;
  task_num_ = 0;
  randnum_ = 0;

  current_task_ = pose_list_["tasks"][task_num_].asString();
  max_num_objects_ = object_list_["num_of_objects"].asInt64();
  max_task_num_ = pose_list_["num_of_tasks"].asInt64();
  wsg_grasp_depth_ = object_list_["grasp_depth"].asDouble();

  iiwa_pose_ = iiwa_pose;
  chosen_plan_ = "";
  object_name_ = "";
  kuka_num_ = kuka_num;

  cabinet_row_pose_.resize(6.0);
  object_state_.resize(6,0);
  pick_pose_.resize(6,0);
  place_pose_.resize(6,0);
}

math::RigidTransformd HighLevelPlanner::NextIiwaMovement(){

  if(current_task_.compare("pick") == 0){
    current_iiwa_state_ = 
      math::RigidTransformd(
        math::RollPitchYawd(pick_pose_[0],
                            pick_pose_[1],
                            pick_pose_[2]),
            Eigen::Vector3d(pick_pose_[3],
                            pick_pose_[4], 
                            pick_pose_[5]));
  }
  else if(current_task_.compare("place") == 0){
    current_iiwa_state_ = 
      math::RigidTransformd(
        math::RollPitchYawd(place_pose_[0],
                            place_pose_[1],
                            place_pose_[2]),
            Eigen::Vector3d(place_pose_[3],
                            place_pose_[4], 
                            place_pose_[5]));
  }
  else if(current_task_.compare("front_of_cabinet_row") == 0){
    current_iiwa_state_ = 
      math::RigidTransformd(
        math::RollPitchYawd(cabinet_row_pose_[0],
                            cabinet_row_pose_[1],
                            cabinet_row_pose_[2]),
            Eigen::Vector3d(cabinet_row_pose_[3],
                            cabinet_row_pose_[4], 
                            cabinet_row_pose_[5]));
  }
  else if(current_task_.compare("front_of_cabinet_column") == 0){
    current_iiwa_state_ = 
      math::RigidTransformd(
        math::RollPitchYawd(place_pose_[0],
                            place_pose_[1],
                            place_pose_[2]),
            Eigen::Vector3d(place_pose_[3],
                            place_pose_[4]+0.1, 
                            place_pose_[5]));
  }
  else if(current_task_.compare("close_wsg") == 0 ||
          current_task_.compare("open_wsg") == 0 ||
          current_task_.compare("new_object") == 0){
       // Keep the same state... do nothing
  }
  else{
    current_iiwa_state_ = 
      math::RigidTransformd(
        math::RollPitchYawd(pose_list_[current_task_][0].asDouble(),
                            pose_list_[current_task_][1].asDouble(),
                            pose_list_[current_task_][2].asDouble()),
            Eigen::Vector3d(pose_list_[current_task_][3].asDouble(),
                            pose_list_[current_task_][4].asDouble(), 
                            pose_list_[current_task_][5].asDouble()));
  }
  return current_iiwa_state_;
}

double HighLevelPlanner::NextWsgMovement(){
  if(current_task_.compare("close_wsg") == 0){
    current_wsg_state_ = 80;
  }
  else if(current_task_.compare("open_wsg") == 0){
    current_wsg_state_ = 100;
  }
  return current_wsg_state_;
}

Vector6<double> HighLevelPlanner::NewObject(){
  object_name_ = object_list_["list"][object_num_].asString();
  chosen_plan_ = object_list_[object_name_]["plan_choice"].asString();
  double object_height = 0;

  for(int i = 0; i < 3; i++){
    pick_pose_[i] = 
      object_list_[object_name_]["grasp_plan"][chosen_plan_]["rpy"][i].asDouble(); // rpy
  }

  if(object_list_[object_name_]["grasp_plan"][chosen_plan_]["height"]
      .asString().compare("full") == 0){

    int height_num = 0; // x    
    if(chosen_plan_.compare("y") == 0){
      height_num = 1;
    }   
    else if(chosen_plan_.compare("z") == 0){
      height_num = 2;
    }   

    object_height = object_list_[object_name_]["size"][height_num].asDouble();
    grasp_height_ = object_height/2 - wsg_grasp_depth_;
  }

  pick_pose_[3] = object_state_[3]-iiwa_pose_[3]; // x 1.5    2.0->0 
  pick_pose_[4] = object_state_[4]-iiwa_pose_[4]; // y 0.5    0.1->0.1
  pick_pose_[5] = object_state_[5]-iiwa_pose_[5]+grasp_height_; // z 0.55   0.15->0.15

  // Rotate frame to match kuka's base
  if(kuka_num_ == 0){
    double temp = pick_pose_[3];
    pick_pose_[3] = pick_pose_[4]; 
    pick_pose_[4] = -temp;
  }


  // Only downstairs kuka
  if(kuka_num_ == 0){
    for(int i = 0; i < 6; i++){
      cabinet_row_pose_[i] = 
        pose_list_[object_list_[object_name_]["cabinet_row"].asString()][i].asDouble();
    }
  }
  for(int i = 0; i < 6; i++){
    place_pose_[i] = 
      pose_list_[object_list_[object_name_]["placement"][kuka_num_].asString()][i].asDouble();
    if(i == 5){
      place_pose_[i] += object_height; // Add the height
    }
  }
  return pick_pose_;
}

bool HighLevelPlanner::NextObject(){
  if(object_num_+1 < max_num_objects_){
      object_num_++; //Update for next time this function is called
      task_num_ = 0;
      current_task_ = pose_list_["tasks"][task_num_].asString();
      return true;
  }
  else return false;
}

int HighLevelPlanner::MoveToNewPlan(){
  task_num_ += 1;
  current_task_ = pose_list_["tasks"][task_num_].asString();
  return task_num_;
}

bool HighLevelPlanner::HasMorePlan(){
  return task_num_ < max_task_num_-1 ? true : false;
}

void HighLevelPlanner::Update(Vector6<double> object_state){
  object_state_ = object_state;
}

int HighLevelPlanner::AddOne(){
  randnum_ += 1;
  return randnum_++;
}

// Getters
std::string HighLevelPlanner::GetChosenPlan(){
  return chosen_plan_;
}
std::string HighLevelPlanner::GetCurrentTask(){
  return current_task_;
}
double HighLevelPlanner::GetObjectGraspHeight(){
  return grasp_height_;
}
std::string HighLevelPlanner::GetObjectName(){
  return object_name_;
}
int HighLevelPlanner::GetObjectNum(){
  return object_num_;
}
int HighLevelPlanner::GetCurrentTaskNum(){
  return task_num_;
}
std::vector<std::string> HighLevelPlanner::GetObjectList(){
  std::vector<std::string> temp;
  int t = max_num_objects_;
  for(int i = 0; i < t; i++){
    temp.push_back(object_list_["list"][i].asString());
  }
  return temp;
}

double HighLevelPlanner::Print(){
  return object_list_[object_list_["list"][object_num_].asString()]["size"][1].asDouble()/2; 
}

Json::Value HighLevelPlanner::JsonParser(std::string file_name){
  Json::Reader reader;
  Json::Value root;
  // drake::log()->info("json parser activated.");
  
  // User does not have to specify /home/your_name/ anymore!
  std::ifstream myfile(FindResourceOrThrow(file_name));
  myfile >> root;
  // drake::log()->info("reading json config file completed."); 
  return root;

  // Example of accessing (DO NOT ERASE BELOW)
  // cout << root["name"].asString() << endl;
}


}  // namespace collaboration_station
}  // namespace examples
}  // namespace drake
