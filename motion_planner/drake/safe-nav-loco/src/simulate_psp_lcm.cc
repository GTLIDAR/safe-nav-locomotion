#include <gflags/gflags.h>

#include "drake/safe-nav-loco/include/beliefIOParser.h"
#include "drake/safe-nav-loco/include/phase_space_planner.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_TAMP_waypoint.hpp"

namespace drake {
namespace phase_space_planner {
namespace {

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
  d0 << 0, 0, 1.01, 1.5708, 0.3;
  //HW
  //d0 << 0, 0, 0.985, 0, 0.1;

  //IROS
   //d0 << (4*cellsize)+(15.5*finecell), (5*cellsize)+(6.5*finecell)-0.5, 0.985, 1.5708, 0.1;

  Eigen::Matrix<double, 5, 1> apex0;
  
  //CDC
  //apex0 << d0(0, 0)+0.11, d0(1, 0), d0(2, 0), d0(3, 0), d0(4, 0);
  //jrnl
   apex0 << d0(0, 0)+0.11+0.07, d0(1, 0), d0(2, 0), d0(3, 0), d0(4, 0);
  //HW
   //apex0 << d0(0, 0), d0(1, 0)-0.11-0.08, d0(2, 0), d0(3, 0), d0(4, 0);
   apex0 << d0(0, 0)+0.03, d0(1, 0), d0(2, 0), d0(3, 0), d0(4, 0);


  Eigen::Matrix<double, 3, 1> foot0;
  //CDC
  foot0 << d0(0, 0)+0.135+0.08, d0(1, 0), d0(2,0)-0.985;
  //HW
  //foot0 << d0(0, 0), d0(1, 0)-0.135-0.08, d0(2,0)-0.985;
  foot0 << d0(0, 0)+0.10012+0.03, d0(1, 0), d0(2,0)-1.01;
  


  Eigen::Matrix<double, 3, 1> obs0;
  Eigen::Matrix<double, 3, 1> obs20;
  //CDC
  //obs0 << (4*cellsize)+(cellsize/2), (cellsize*6)+(cellsize/2), 0;
  obs20 << (10*cellsize)+(cellsize/2), (cellsize*10)+(cellsize/2), 0;
  //IROS
  obs0 << (0*cellsize)+(cellsize/2), (cellsize*0)+(cellsize/2), 2.25;
 

  ::phase_space_planner::PhaseSpacePlanner psp;
  psp.Init(apex0, d0, foot0);
  psp.InitObstacle(obs0,obs20);

  

  drake::lcm::DrakeLcm lcm;
  const std::string channel_waypoint = "new_waypoint";
  drake::lcm::Subscriber<lcmt_TAMP_waypoint> subscription(&lcm, channel_waypoint);
  const lcmt_TAMP_waypoint& msg_x = subscription.message();


    {
      lcmt_TAMP_waypoint initial_action{};
      initial_action.N = 1;
      initial_action.stepH = 0;
      initial_action.turn = 2;
      initial_action.stop = 1;
      initial_action.forward = 
      initial_action.stanceFoot= 0;
      Publish(&lcm, channel_waypoint, initial_action);
    }
     while(!(lcm.HandleSubscriptions(0))) //HOLD FOR NEW Action
    {
    
    }

  double v;
  double pre_v = 0.1;

  for (int i = 0; i < 120; i++) 
  {

     while(!(lcm.HandleSubscriptions(0))) //HOLD FOR NEW Action
    {
    
    }

    // revieve msg.x
    //lcmt_TAMP_waypoint rec{};
    //rec.N = msg_x.N;
    //std::vector<int> obstacle_location = parser.getPropertyArray("obstacle_location");
    Eigen::Matrix<double, 3, 1> obs;
    Eigen::Matrix<double, 3, 1> obs2;
    obs << 0,0,0;//(obstacle_location[0]*(cellsize/10))-cellsize/2, (obstacle_location[1]*(cellsize/10))-cellsize/2, 2.25;
    obs2 << 0,0,0;//((obstacle_location[2])*(cellsize/10))-cellsize/2, ((obstacle_location[3])*(cellsize/10))-cellsize/2+(2*cellsize), 0;
 
    double h =1.01 ;//0.985;
    /*
    int stepL = parser.getProperty("stepL");
    int stepH = parser.getProperty("stepH");
    int turn = parser.getProperty("turn");
    int stop = parser.getProperty("stop");
    int forward = parser.getProperty("forward");
    int stanceFoot = parser.getProperty("stanceFoot");
    */
    int stepL = msg_x.stepL;//  parser.getProperty("stepL");
    int stepH = msg_x.stepH; //parser.getProperty("stepH");
    int turn = msg_x.turn; //parser.getProperty("turn");
    int stop = msg_x.stop; //parser.getProperty("stop");
    int forward =msg_x.forward; //parser.getProperty("forward");
    int stanceFoot =msg_x.stanceFoot;// parser.getProperty("stanceFoot");
    
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

    double step_factor =0.3;

    



    if (stepH == 3)
    {

      dheight = 0;

    }
    else if (stepH == 2)
    {
      
      dheight = -0.1;

      v = pre_v + 0.05;
      if (pre_v >= 0.6)
      {
        v = 0.6;
      }
    }
        else if (stepH == 4)
    {
      
      dheight = 0.1;

      v = pre_v + 0.05;
      if (pre_v >= 0.6)
      {
        v = 0.6;
      }
    }


    if (turn == 0)
    {
      dheading =0.3926991;// 0.261799;//
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

      dheading = -0.3926991;//-0.261799;//-0.3926991;
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
      step_length = 0.3119+0.104;//0.55;0.3119+0.104; //4 fine cells
      //v= 0.45;
      v = pre_v + 0.05;
      if (pre_v >= 0.6)
      {
        v = 0.6;
      }
    }
    else if (stepL == 1)
    {
      step_length = 0.28; //3 fine cells
    }
    else if (stepL == 2)
    {
      step_length = 0.43; //4 fine cells
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
      step_length =  .207962;
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
        Primitive acn(step_length*step_factor, 0, dheight, v, h); 
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
        Primitive acn(step_length*step_factor, dheading, dheight, 0.1, h);
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

        Primitive acn(step_length*step_factor, dheading, dheight, v, h);
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
        
        Eigen::Matrix<double, 13, 1> psp_param;
        psp_param << 0, 0, 0, 0.25 ,0.4, 0, 0.2, 0.2, 0.1, 0, 0, 0, 0;
        psp.psp_list.push_back(psp_param);
        // Primitive acn(0.02, 0, 0, 0.05, h);
        // psp.UpdatePrimitive(acn);
        // psp.UpdateKeyframe();
        // //psp.Stand(obs, obs2);
        //v = 0.1;
      
    }
    //parser.advanceStep();
    pre_v = v;

    //std::cout << "N= " << psp.N << "S= " << psp.S << "E= " << psp.E << "W= " << psp.W <<  std::endl << std::endl;
    {
      lcmt_TAMP_waypoint dummy{};
      dummy.N = psp.N;
      dummy.S = psp.S;
      dummy.E = psp.E;
      dummy.W = psp.W;
      Publish(&lcm, channel_waypoint, dummy);
    }

    while(!(lcm.HandleSubscriptions(0))) //HOLD FOR NEW MSG
    {
    }

    
    // revieve msg.x
    //lcmt_TAMP_waypoint rec{};
    //rec.N = msg_x.N;
    //std::cout << "psp.N = " << psp.N << "rec.N= " << rec.N << std::endl << std::endl << std::endl ;
/*
    {
      lcmt_TAMP_waypoint all_action{};
      all_action.stepL = 0;
      all_action.stepH = 0;
      all_action.turn = 2;
      all_action.stop = 0;
      all_action.forward = 1;
      all_action.stanceFoot= stanceFoot;
      Publish(&lcm, channel_waypoint, all_action);
    }
*/
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
  write_data<13>(psp.psp_list, file_name);
  
  return 0;
}

}  // namespace
}  // namespace phase_space_planner
} //namesoace drake
int main(int argc, char* argv[]) 
{
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::phase_space_planner::DoMain();
}