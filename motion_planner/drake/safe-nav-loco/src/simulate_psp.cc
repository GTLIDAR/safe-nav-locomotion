#include <gflags/gflags.h>

#include "drake/safe-nav-loco/include/beliefIOParser.h"
#include "drake/safe-nav-loco/include/phase_space_planner.h"

namespace phase_space_planner
{
namespace
{

template <int Derived>
int read_data(std::vector<Eigen::Matrix<double, Derived, 1>>& data, std::string& file_name) 
{
  std::ifstream inFile;
  inFile.open(file_name, std::ios::in);
  std::string line;
	while (getline(inFile, line))
  {
    std::istringstream linestream(line);
    std::vector<std::string> vv;
    std::string v;
    while (getline(linestream, v, ' ')) 
    {
        vv.push_back(v);
    }

    Eigen::Matrix<double, Derived, 1> dd;
    for (int i = 0; i < Derived; i++)
    {
      dd(i, 0) = std::atof(vv[i].c_str());
    }
    data.push_back(dd);
  }
  inFile.close();

  return 1;
}

template <int Derived>
int write_data(std::vector<Eigen::Matrix<double, Derived, 1>>& data, std::string& file_name) 
{
  std::ofstream outFile;
  outFile.open(file_name, std::ios::out);
  int data_size = data.size();
  for (int n = 0; n < data_size; n++)
  {
    for (int i = 0; i < Derived; i++)
    {
      outFile << std::fixed << std::setprecision(8) << data[n](i, 0) << " ";
    }
    outFile << std::endl;
  }
  outFile.close();

  return 1;
}

int DoMain()
{
  Eigen::Matrix<double, 5, 1> d0;
  double cellsize= 2.70351;
  //CDC2020 Start Position 
  d0 << (4*cellsize)+(cellsize/2), (cellsize*9)+(cellsize/2), 0.8+0.6, 1.5708, 0.1;


  Eigen::Matrix<double, 5, 1> apex0;
  
  //CDC
  apex0 << d0(0, 0)+0.11, d0(1, 0), d0(2, 0), d0(3, 0), d0(4, 0);


  Eigen::Matrix<double, 3, 1> foot0;
  //CDC
  foot0 << d0(0, 0)+0.135, d0(1, 0), d0(2,0)-0.8;
  


  Eigen::Matrix<double, 3, 1> obs0;
  //CDC
  obs0 << -(14*0.2+0.1), 2*0.2+0.1, 0;
 

  PhaseSpacePlanner psp;
  psp.Init(apex0, d0, foot0);
  psp.InitObstacle(obs0);


  BeliefIOParser parser("/home/sa-zhao/code/safe-nav-locomotion/motion_planner/drake/safe-nav-loco/vis/actions_CDC_Sub.json");
  parser.advanceStep();
  double v;
  double pre_v = 0.1;

  for (int i = 0; i < 170; i++) 
  {
    std::vector<int> obstacle_location = parser.getPropertyArray("obstacle_location");
    Eigen::Matrix<double, 3, 1> obs;
    obs << -(obstacle_location[1]*0.2+0.1), obstacle_location[0]*0.2+0.1, 0;
 

    int stepL = parser.getProperty("stepL");
    int stepH = parser.getProperty("stepH");
    int turn = parser.getProperty("turn");
    int stop = parser.getProperty("stop");
    int forward = parser.getProperty("forward");
    int stanceFoot = parser.getProperty("stanceFoot");
    
    if (stanceFoot == 0)
    {
      stanceFoot = 1;
    }
    else
    {
      stanceFoot = 0;
    }
    

    double step_length = 0;
    double dheading = 0;
    double dheight = 0;

    if (stepL == 0)
    {
      step_length = 0.3119+0.104;
    }
    else if (stepL == 1)
    {
      step_length = 0.28;
    }
    else if (stepL == 2)
    {
      step_length = 0.43;
    }
    else if (stepL == 3)
    {
      step_length = 0.3839;
    }



    if (stepH == 3)
    {

      dheight = 0;
      v = pre_v + 0.05;
      if (pre_v >= 0.45)
      {
        v = 0.45;
      }
      std::cout << "flat: ";
    }
    else if (stepH == 2)
    {
      
      dheight = -0.1;
      std::cout << "down: ";
      v = pre_v + 0.05;
      
      if (pre_v > 0.4)
      {
        v = 0.4;
      }
    }
    
    if (turn == 0)
    {
      dheading = 0.3926991;
      std::cout << "Turn left 22.5: ";
      if (v > 0.3)
      {
        pre_v = 0.25;
        v = pre_v - 0.025;
        if (v < 0.2)
        {
         v = 0.2;
        }

      }

    }
    else if (turn == 1)
    {
      dheading = 0;      
    }
    else if (turn == 2)
    { 

      dheading = -0.3926991;
      std::cout << "Turn Right 22.5: ";

      if (v > 0.3)
      {
        pre_v = 0.25;
        v = pre_v - 0.025;
        if (v < 0.2)
        {
         v = 0.2;
        }

      }
    }
   
    
    std::cout << "Step: " << i << std::endl;
    if (forward == 1)
    {
      if (psp.stance == 2)
      { 
     
        std::cout << "Start: ";
        psp.Start(stanceFoot);
        Primitive acn(step_length, 0, dheight, v, 0.8); 
        psp.UpdatePrimitive(acn);
        psp.UpdateKeyframe();
        psp.UpdateObstacle(obs);
        psp.FirstStep();
    
        std::cout << "vapex:" << v << std::endl;
        
      }
      else if (stop == 1)
      { 
        std::cout << "Stop: ";
        v = 0.1;
        Primitive acn(step_length, dheading, dheight, 0.1, 0.8);
        psp.UpdatePrimitive(acn);
        psp.UpdateKeyframe();
        psp.UpdateObstacle(obs);
        psp.UpdateTrajectory();
        psp.LastStep();
        psp.End();
        std::cout << "vapex:" << v << std::endl;
      }
      else
      {
        std::cout << "update: ";
        Primitive acn(step_length, dheading, dheight, v, 0.8);
        psp.UpdatePrimitive(acn);
        psp.UpdateKeyframe();
        psp.UpdateObstacle(obs);
        psp.UpdateTrajectory();
        std::cout << "vapex:" << v << std::endl;

      }
    }
    else 
    { 

        psp.Stand(obs);
        v = 0.1;
      
    }
    parser.advanceStep();
    pre_v = v;
  }
   
  // Log 
  std::string file_name;
  
  file_name = "/home/sa-zhao/code/safe-nav-locomotion/motion_planner/drake/safe-nav-loco/vis/log_apex.txt";
  write_data<5>(psp.apex_list, file_name);

  file_name = "/home/sa-zhao/code/safe-nav-locomotion/motion_planner/drake/safe-nav-loco/vis/log_d.txt";
  write_data<5>(psp.d_list, file_name);

  file_name = "/home/sa-zhao/code/safe-nav-locomotion/motion_planner/drake/safe-nav-loco/vis/log_switch.txt";
  write_data<6>(psp.switch_list, file_name);

  file_name = "/home/sa-zhao/code/safe-nav-locomotion/motion_planner/drake/safe-nav-loco/vis/log_p_foot.txt";
  write_data<3>(psp.p_foot_list, file_name);

  file_name = "/home/sa-zhao/code/safe-nav-locomotion/motion_planner/drake/safe-nav-loco/vis/log_COM.txt";
  write_data<9>(psp.COM_list, file_name);

  file_name = "/home/sa-zhao/code/safe-nav-locomotion/motion_planner/drake/safe-nav-loco/vis/log_l_foot.txt";
  write_data<9>(psp.l_foot_list, file_name);

  file_name = "/home/sa-zhao/code/safe-nav-locomotion/motion_planner/drake/safe-nav-loco/vis/log_r_foot.txt";
  write_data<9>(psp.r_foot_list, file_name);

  file_name = "/home/sa-zhao/code/safe-nav-locomotion/motion_planner/drake/safe-nav-loco/vis/log_heading.txt";
  write_data<1>(psp.heading_list, file_name);

  file_name = "/home/sa-zhao/code/safe-nav-locomotion/motion_planner/drake/safe-nav-loco/vis/log_obstacle.txt";
  write_data<3>(psp.obstacle_list, file_name);
  
  return 0;
}

}  // namespace
}  // namespace phase_space_planner

int main(int argc, char* argv[]) 
{
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return phase_space_planner::DoMain();
}