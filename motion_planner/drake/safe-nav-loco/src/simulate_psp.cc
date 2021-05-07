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
  float cellsize= 2.70351;
  //iros
    //cellsize = 2.39;
    //float finecell = 0.103981;
  //CDC2020 Start Position 
  //d0 << (4*cellsize)+(cellsize/2), (cellsize*9)+(cellsize/2), 0.985+0.6, 1.5708, 0.1;
  //jrnl
  d0 << (3.5*cellsize)-0.2, (9.5*cellsize)-0.5, 0.985+1.2, 1.5708, 0.1;
  //HW
  //d0 << 0, 0, 0.985, 0, 0.1;

  //IROS
   //d0 << (4*cellsize)+(15.5*finecell), (5*cellsize)+(6.5*finecell)-0.5, 0.985, 1.5708, 0.1;

  Eigen::Matrix<double, 5, 1> apex0;
  
  //CDC
  //apex0 << d0(0, 0)+0.11, d0(1, 0), d0(2, 0), d0(3, 0), d0(4, 0);
  //jrnl
   apex0 << d0(0, 0)+0.11+0.08, d0(1, 0), d0(2, 0), d0(3, 0), d0(4, 0);
  //HW
   //apex0 << d0(0, 0), d0(1, 0)-0.11-0.08, d0(2, 0), d0(3, 0), d0(4, 0);


  Eigen::Matrix<double, 3, 1> foot0;
  //CDC
  foot0 << d0(0, 0)+0.135+0.08, d0(1, 0), d0(2,0)-0.985;
  //HW
  //foot0 << d0(0, 0), d0(1, 0)-0.135-0.08, d0(2,0)-0.985;
  


  Eigen::Matrix<double, 3, 1> obs0;
  Eigen::Matrix<double, 3, 1> obs20;
  //CDC
  //obs0 << (4*cellsize)+(cellsize/2), (cellsize*6)+(cellsize/2), 0;
  obs20 << (10*cellsize)+(cellsize/2), (cellsize*10)+(cellsize/2), 0;
  //IROS
  obs0 << (0*cellsize)+(cellsize/2), (cellsize*0)+(cellsize/2), 2.25;
 

  PhaseSpacePlanner psp;
  psp.Init(apex0, d0, foot0);
  psp.InitObstacle(obs0,obs20);


  BeliefIOParser parser("/home/sa-zhao/code/safe-nav-locomotion/motion_planner/drake/safe-nav-loco/vis/Belief_Evasion_fine_abstraction_straight.json");
  parser.advanceStep();
  double v;
  double pre_v = 0.1;

  for (int i = 0; i < 60; i++) 
  {
    std::vector<int> obstacle_location = parser.getPropertyArray("obstacle_location");
    Eigen::Matrix<double, 3, 1> obs;
    Eigen::Matrix<double, 3, 1> obs2;
    obs << (obstacle_location[0]*(cellsize/10))-cellsize/2, (obstacle_location[1]*(cellsize/10))-cellsize/2, 2.25;
    obs2 << ((obstacle_location[2])*(cellsize/10))-cellsize/2, ((obstacle_location[3])*(cellsize/10))-cellsize/2+(2*cellsize), 0;
 
    double h = 0.985;
    
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

    



    if (stepH == 3)
    {

      dheight = 0;
   //   v = pre_v + 0.05;
    //  if (pre_v >= 0.45)
    //  {
    //    v = 0.45;
    //  }
     // std::cout << "flat: ";
    }
    else if (stepH == 2)
    {
      
      dheight = -0.1;
     // std::cout << "down: ";
     /*
      v = pre_v + 0.05;
      
      if (pre_v > 0.4)
      {
        v = 0.4;
      }
      */
      v = pre_v + 0.05;
      if (pre_v >= 0.6)
      {
        v = 0.6;
      }
    }
        else if (stepH == 4)
    {
      
      dheight = 0.1;
     // std::cout << "down: ";
     /*
      v = pre_v + 0.05;
      
      if (pre_v > 0.4)
      {
        v = 0.4;
      }
      */
      v = pre_v + 0.05;
      if (pre_v >= 0.6)
      {
        v = 0.6;
      }
    }
    /*
    if ( turn == 2)
    {
      turn = 3;
    }
    if ( turn == 1)
    {
      turn = 2;
    }

  */


    if (turn == 0)
    {
      dheading = 0.261799;//0.3926991;
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
      //v =0.2;

    }
        else if (turn == 1)
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
      //v=0.2;    
    }

    else if (turn == 2)
    {
      dheading = 0;      
    }
        else if (turn == 22)
    {
      dheading = 0.00001;      
    }


    else if (turn == 3)
    { 

      dheading = -0.261799;//-0.3926991;
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
      //v=0.2;
    }
    else if (turn == 4)
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

     // v=0.2;
      
    }
   

    if (stepL == 0)
    {
      step_length = 0.3119+0.104;//0.55;0.3119+0.104;
      //v= 0.45;
      v = pre_v + 0.05;
      if (pre_v >= 0.6)
      {
        v = 0.6;
      }
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
      //step_length = 0.265935;
    }
    else if (stepL == 4)
    {
      //step_length = 0.225;
      step_length = 0.47389;
      //v=0.2;
    }
    else if (stepL == 5)
    {
      step_length = .207962;
      //v=0.3;
    }
    else if (stepL == 6)
    {
      step_length = .311944;
     //v =0.2;
    }
    else if (stepL == 7)
    {
      step_length = .519906;
      //v =0.3;
    }
    else if (stepL == 8) //HW
    {
      step_length = 0.2;
      //v =0.3;
    }
    else if (stepL == 9)
    {
      step_length = 0.3119+0.104+0.104;

        v = pre_v + 0.05;
      if (pre_v >= 0.7)
      {
        v = 0.7;
      }
      
    }
        else if (stepL == 99)
    {
      step_length = 0.104*7;

        v = pre_v + 0.05;
      if (pre_v >= 0.9)
      {
        v = 0.9;
      }
      
    }
    else if (stepL == 91)
    {
      step_length = 0.104;

     v = pre_v + 0.05;
      if (pre_v >= 0.45)
      {
        v = 0.45;
      }
    }
        else if (stepL == 92)
    {
      step_length = 0.104*2;

     v = pre_v + 0.05;
      if (pre_v >= 0.45)
      {
        v = 0.45;
      }
      
    }
        else if (stepL == 93)
    {
      step_length = 0.104*3;

     v = pre_v + 0.05;
      if (pre_v >= 0.35)
      {
        v = 0.35;
      }
      
    }
        else if (stepL == 94)
    {
      step_length = 0.104*4;

     v = pre_v + 0.05;
      if (pre_v >= 0.6)
      {
        v = 0.6;
      }
      
    }
        else if (stepL == 95)
    {
      step_length = 0.104*5;
     
     v = pre_v + 0.05;
      if (pre_v >= 0.6)
      {
        v = 0.6;
      }
      //v = 0.6;
    }


    
    
     //HW
    /*
     v = 0.3;
     step_length =  0.15;
     if (i > 25)
     {
     v = 0.45;
     step_length =  0.19;
     }

    if (i > 35)
     {
     v = pre_v+0.05;
     if (v >= 0.6)
     {
      v = 0.6;
     }
     step_length =  0.3;
     }
     if (i > 50)
     {
      v = 0.3;
      step_length =  0.15;
     }
    */
     //dheading = 0;
     //v=0.45;
     //step_length = 0.18;
    //std::cout << "PSP.vapex " << psp.X_apex(4,0) << std::endl;
    /*
    if ( i == 16)
    {
      v = -0.2;
    }
    if ( i > 16)
    {
      v = -v;
      step_length = -step_length;
    }
    */
    //std::cout << "Step: " << i << std::endl;
    if (forward == 1)
    {
      if (psp.stance == 2)
      { 
        
        //std::cout << "vapex:" << v << std::endl;
        //std::cout << "Start: ";
        psp.Start(stanceFoot);
        Primitive acn(step_length, 0, dheight, v, h); 
        psp.UpdatePrimitive(acn);
        psp.UpdateKeyframe();
        psp.UpdateObstacle(obs, obs2);
        psp.FirstStep();
        //psp.UpdateTrajectory();
    
        
        
      }
      else if (stop == 1)
      { 

        //std::cout << "Stop: ";
        //v = 0.1;
       // std::cout << "vapex:" << v << std::endl;
        Primitive acn(step_length, dheading, dheight, 0.1, h);
        psp.UpdatePrimitive(acn);
        psp.UpdateKeyframe();
        psp.UpdateObstacle(obs, obs2);
        psp.UpdateTrajectory();
        psp.LastStep();
        psp.End();
        
      }
      else
      {
       // std::cout << "vapex:" << v << std::endl;
        //std::cout << "update: ";

        Primitive acn(step_length, dheading, dheight, v, h);
        psp.UpdatePrimitive(acn);
        psp.UpdateKeyframe();
        psp.UpdateObstacle(obs, obs2);
        psp.UpdateTrajectory();

        //if (psp.goodlateral(0,0) == 0)
        //{

       // }
        

      }
    }
    else 
    { 

        psp.Stand(obs, obs2);
        //v = 0.1;
      
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
    file_name = "/home/sa-zhao/code/safe-nav-locomotion/motion_planner/drake/safe-nav-loco/vis/log_waypoint.txt";
  write_data<5>(psp.waypoint_list, file_name);

  file_name = "/home/sa-zhao/code/safe-nav-locomotion/motion_planner/drake/safe-nav-loco/vis/log_switch.txt";
  write_data<6>(psp.switch_list, file_name);

  file_name = "/home/sa-zhao/code/safe-nav-locomotion/motion_planner/drake/safe-nav-loco/vis/log_apextraj.txt";
  write_data<6>(psp.apextraj_list, file_name);


  file_name = "/home/sa-zhao/code/safe-nav-locomotion/motion_planner/drake/safe-nav-loco/vis/log_p_foot.txt";
  write_data<3>(psp.p_foot_list, file_name);

  file_name = "/home/sa-zhao/code/safe-nav-locomotion/motion_planner/drake/safe-nav-loco/vis/log_COM.txt";
  write_data<10>(psp.COM_list, file_name);

  file_name = "/home/sa-zhao/code/safe-nav-locomotion/motion_planner/drake/safe-nav-loco/vis/log_l_foot.txt";
  write_data<10>(psp.l_foot_list, file_name);

  file_name = "/home/sa-zhao/code/safe-nav-locomotion/motion_planner/drake/safe-nav-loco/vis/log_r_foot.txt";
  write_data<10>(psp.r_foot_list, file_name);

  file_name = "/home/sa-zhao/code/safe-nav-locomotion/motion_planner/drake/safe-nav-loco/vis/log_heading.txt";
  write_data<1>(psp.heading_list, file_name);

  file_name = "/home/sa-zhao/code/safe-nav-locomotion/motion_planner/drake/safe-nav-loco/vis/log_obstacle.txt";
  write_data<6>(psp.obstacle_list, file_name);

  file_name = "/home/sa-zhao/code/safe-nav-locomotion/motion_planner/drake/safe-nav-loco/vis/T_step.txt";
  write_data<3>(psp.Tlist, file_name);
  
  file_name = "/home/sa-zhao/code/safe-nav-locomotion/motion_planner/drake/safe-nav-loco/vis/log_delta.txt";
  write_data<3>(psp.sim_list, file_name);

  file_name = "/home/sa-zhao/code/safe-nav-locomotion/motion_planner/drake/safe-nav-loco/vis/log_step.txt";
  write_data<2>(psp.step_list, file_name);

   file_name = "/home/sa-zhao/code/safe-nav-locomotion/motion_planner/drake/safe-nav-loco/vis/log_psp.txt";
  write_data<8>(psp.psp_list, file_name);
  
  return 0;
}

}  // namespace
}  // namespace phase_space_planner

int main(int argc, char* argv[]) 
{
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return phase_space_planner::DoMain();
}