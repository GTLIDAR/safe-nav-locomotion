#include "drake/safe-nav-loco/include/phase_space_planner.h"

#include <iostream>
#include <fstream>


namespace phase_space_planner
{

  Eigen::Matrix<double, 3, 1> PhaseSpacePlanner::ForwardProp(
      double p_f, double p, double p_dot, 
      double h, double aq, double eps, double count) 
  {
    double w_sq = G / h; 
    double p_ddot;
    double inc_p;
    Eigen::Matrix<double, 3, 1> result;

    for (int i = 0; i < count; i++)
    {   
     
      p_ddot = w_sq * (p - p_f);
      inc_p = eps * p_dot + 0.5 * eps * eps * p_ddot;
      p = p + inc_p;
      p_dot = p_dot + eps * p_ddot;



    }

    result(0, 0) = p;
    result(1, 0) = p_dot;
    result(2, 0) = p_ddot;

    return result;
  }

  void PhaseSpacePlanner::Init(Eigen::Matrix<double, 5, 1>& apex_init,
                               Eigen::Matrix<double, 5, 1>& d_init,
                               Eigen::Matrix<double, 3, 1>& p_foot_init)
  { 

    X_apex = apex_init;
    X_d = d_init;
    p_foot = p_foot_init;
    waypoint = X_d;
    apex_list.push_back(X_apex);
    d_list.push_back(X_d);
    waypoint_list.push_back(waypoint);
    p_foot_list.push_back(p_foot);
    

    Eigen::Matrix<double, 10, 1> COM;
    COM << X_apex(0, 0), X_apex(1, 0), X_apex(2, 0), 0, 0, 0, 0, 0, 0, 0;
    COM_list.push_back(COM);
    Eigen::Matrix<double, 10, 1> l_foot;
    l_foot << X_apex(0, 0) + 0.135 * std::sin(-X_apex(3, 0)),
              X_apex(1, 0) + 0.135 * std::cos(-X_apex(3, 0)), 
              p_foot(2, 0), 0, 0, 0, 0, 0, 0, 0;
    l_foot_list.push_back(l_foot);
    Eigen::Matrix<double, 10, 1> r_foot;
    r_foot << X_apex(0, 0) - 0.135 * std::sin(-X_apex(3, 0)),
              X_apex(1, 0) - 0.135 * std::cos(-X_apex(3, 0)), 
              p_foot(2, 0), 0, 0, 0, 0, 0, 0, 0;
    r_foot_list.push_back(r_foot);
    Eigen::Matrix<double, 1, 1> heading;
    heading << X_apex(3, 0);
    heading_list.push_back(heading);

    startflag = 1 ;





  }


  void PhaseSpacePlanner::UpdatePrimitive(Primitive& primitive) 
  {
    prim(0, 0) = primitive.step_length;
    prim(1, 0) = primitive.dheading;
    prim(2, 0) = primitive.dheight;
    prim(3, 0) = primitive.v_apex;
    prim(4, 0) = primitive.h_apex;

  }

  void PhaseSpacePlanner::UpdateKeyframe() 
  { 
    
    double COS = std::cos(X_apex(3, 0) + prim(1, 0));
    double SIN = std::sin(X_apex(3, 0) + prim(1, 0));

    double s1, s1_dot, s1_ddot, l1, l1_dot,l1_ddot, v1,v1_dot, v1_ddot, s1_foot, l1_foot, v1_foot;
    //double sign;
    double cost = 10000000;
    double new_cost;
    double s2;
    double s2_dot, s2_ddot, l2, l2_dot, l2_ddot, v2, v2_dot, v2_ddot;
    double phi = 0 ;//- waypoint(3,0);
    // Foot2
    double s2_foot, l2_foot, v2_foot;

    double opt_vn;
       
    double fine_num=0;
    // Switch
    double s_switch, ds_switch, l_switch, dl_switch, v_switch, dv_switch;

    // Forward-Backward Prop for Sagittal Switch
    double eps, forward_num, backward_num, aq, bq, w1_sq, w2_sq;
    double temp_s2_dot;
    double sum_vn=0;
    int sweep_c = 0;
    double v_inc[60];
   
      temp_s2_dot = prim(3, 0);  
      double good_vn[60];
      

      //double v_inc[] = {0.1,1.25, 0.15,1.75, 0.2,.225, 0.25,.275, 0.3,.325, 0.35,.375, 0.4,.425, 0.45,.475, 0.5,.525, 0.55,.575, 0.6,.625, 0.65};
      for (int i=0; i<=60; i++)
      {
        v_inc[i]=(i*0.01)+0.1;
      }
     //goodlateral = 1;
      //goodlateral(0,0) = 0;
      //std::cout << " og_vn= " << prim(3, 0) << std::endl;
      for (int counter = 0; counter <= 60; counter++)
      { 
        //std::cout << "counter= "<< counter  << std::endl;

       //if (goodlateral == 0)
       // {
           // if (prim(1, 0) ==0) //walking straight
          // {

              prim(3, 0) = v_inc[counter]; //go through the apex velocities
              if (counter == 60) // if done with the for loop, do an extra one but chose the mid-point
              {
                

                  prim(3, 0) = good_vn[((sweep_c-1)/2)];
                  prim(3, 0) = opt_vn;
     
                  phi = 0 ;//- waypoint(3,0);
     
              }
            //}
           // else //if turning exit the for loop and don't change the assigned apex velocity
           // {
           ///   counter = 61;
           //   prim(3, 0) = temp_s2_dot; //use assigned v_apex
           //  goodlateral(0,0) = 1;
           //   phi = 0 ;//- waypoint(3,0);
              
          //  }

       //}  

        //std::cout << "searching = "<< prim(3,0)  << std::endl;
          s1 = (X_apex(0, 0) - X_d(0, 0))*COS + 
                      (X_apex(1, 0) - X_d(1, 0))*SIN;
          s1_dot = X_apex(4, 0)*std::cos(prim(1, 0)-phi);
          

          l1 = -(X_apex(0, 0) - X_d(0, 0))*SIN + 
                       (X_apex(1, 0) - X_d(1, 0))*COS;
          l1_dot = -X_apex(4, 0)*std::sin(prim(1, 0)-phi);
          
          
          //std::cout <<  "cos= "<< COS << "sin= " << SIN << std::endl;

          v1 = X_apex(2, 0);

          // Foot1 adjustment to new local coordinates
           
          if (startflag == 1)
          { 
            if (stance == 1)
            {
              s1_foot = (p_foot(0, 0) - X_d(0, 0))*COS + (p_foot(1, 0) - X_d(1, 0))*SIN;
              l1_foot = (p_foot(0, 0) - X_d(0, 0))*SIN + (p_foot(1, 0) - X_d(1, 0))*COS;
              v1_foot = p_foot(2, 0); 


            }
            else
            {
              s1_foot = (p_foot(0, 0) - X_d(0, 0))*COS + (p_foot(1, 0) - X_d(1, 0))*SIN;
              l1_foot = -(p_foot(0, 0) - X_d(0, 0))*SIN + (p_foot(1, 0) - X_d(1, 0))*COS;
              v1_foot = p_foot(2, 0); 


            }
            
            startflag = 0 ;
          }
          else
          {
            s1_foot = (p_foot(0, 0) - X_d(0, 0))*COS + (p_foot(1, 0) - X_d(1, 0))*SIN;
            l1_foot = -(p_foot(0, 0) - X_d(0, 0))*SIN + (p_foot(1, 0) - X_d(1, 0))*COS;
            v1_foot = p_foot(2, 0); 
          }
         
          

          
           

           // Keyframe2
         // if (prim(1,0) > 0)
          //{
          //  sign = -1;
            // s2 = s1 + prim(0, 0) + sign*((X_apex(0, 0) - X_d(0, 0))*std::cos(X_apex(3, 0) + prim(1, 0)+phi) + (X_apex(1, 0) - X_d(1, 0))*std::sin(X_apex(3, 0) + prim(1, 0)+phi))  ; 
           //  s2_foot = s2;
            // s2 = s1 + prim(0, 0) + sign*((X_apex(0, 0) - waypoint(0, 0))*COS + (X_apex(1, 0) - waypoint(1, 0))*SIN)  ; 
             //s2_foot = s2;
          //}
          //else if (prim(1,0) < 0)
         // {
           // sign = -1;
            // s2 = s1 + prim(0, 0) + sign*((X_apex(0, 0) - X_d(0, 0))*std::cos(X_apex(3, 0) + prim(1, 0)+phi)+ (X_apex(1, 0) - X_d(1, 0))*std::sin(X_apex(3, 0) + prim(1, 0)+phi))  ; 
            // s2_foot = s2;
             //s2 = s1 + prim(0, 0) + sign*((X_apex(0, 0) - waypoint(0, 0))*COS + (X_apex(1, 0) - waypoint(1, 0))*SIN)  ; 
             //s2_foot = s2;
         // }
         // else
          //{
  
            
            //s2 = s1 + prim(0, 0);
            //s2_foot = s2;
            s2 = s1 + prim(0, 0) - ((X_apex(0, 0) - X_d(0, 0))*std::cos(X_apex(3, 0) + prim(1, 0)+phi) + (X_apex(1, 0) - X_d(1, 0))*std::sin(X_apex(3, 0) + prim(1, 0)+phi))  ; 
            //s2_foot = s2;

          //}
          

          s2_dot = prim(3, 0);
                    
          l2_dot = 0;
          
          
          v2 = v1_foot + prim(2, 0) +  prim(4, 0); 
          
          //std::cout << "delta y2=" << l1_foot - l1 << std::endl;
          //std::cout << "delta y1=" << l1 << std::endl;
          // Foot2
          s2_foot = s2;
          // std::cout << "s2= " << s2 << " s2_foot= "<< s2_foot << std::endl;

          //double dis = 0;
          if (stance == 1)
          {
            l2_foot = -0.135;

          }
          else
          {
            l2_foot = 0.135 ;

          }
               
          // Forward-Backward Prop for Sagittal Switch
          eps = 0.001;
          forward_num = 0;
          backward_num = 0;
          aq = (v2 - v1) / (s2 - s1); 
          bq = 0;
          w1_sq = G / (prim(4, 0));// G / (aq*(s1_foot) + bq*(l1_foot) + prim(4, 0));
          w2_sq = G / (prim(4, 0));//G / (aq*(s2_foot) + bq*(l2_foot) + prim(4, 0));
          double inc_s1, inc_l1, inc_s2;
          
        
          while (std::abs(s2) > std::abs(s1)) 
          {
            // Forward Prop
            if (std::abs(s1_dot) < std::abs(s2_dot)) 
            {
              forward_num = forward_num + 1;
              s1_ddot = w1_sq * (s1 - s1_foot);
              inc_s1 = eps * s1_dot + 0.5 * eps * eps * s1_ddot;
              s1 = s1 + inc_s1;
              s1_dot = s1_dot + eps * s1_ddot;

              l1_ddot = w1_sq * (l1 - l1_foot);
              inc_l1 = eps * l1_dot + 0.5 * eps * eps * l1_ddot;
              l1 = l1 + inc_l1;
              l1_dot = l1_dot + eps * l1_ddot;

              v1_ddot = aq * s1_ddot + bq * l1_ddot;
              v1 = v1 + aq * inc_s1 + bq * inc_l1;
              v1_dot = aq * s1_dot + bq * l1_dot;

              
               //std::cout << "forward_num "<< std::endl;
            } 
            // Backward Prop
            else 
            {
              backward_num = backward_num + 1;
              s2_ddot = w2_sq * (s2 - s2_foot);
              inc_s2 = - eps * s2_dot - 0.5 * eps * eps * s2_ddot;
              s2 = s2 + inc_s2;
              s2_dot = s2_dot - eps * s2_ddot;

              v2_ddot = aq * s2_ddot;
              v2 = v2 + aq * inc_s2;
              v2_dot = aq * s2_dot;

              
              //std::cout << "backward" << std::endl;
            }
          }
          //std::cout << "backward_num " << backward_num << std::endl;
          //std::cout << "forward_num " << forward_num << std::endl;
          s_switch = (s1 + s2) / 2;
          ds_switch = (s1_dot + s2_dot) / 2;

          l_switch = l1;
          dl_switch = l1_dot;

          v_switch = (v1 + v2) / 2;
          dv_switch = (v1_dot + v2_dot) / 2;


          // Newton-Raphson Search for Lateral Foot Placement
          int n = 1, n_max = 150;
          double max_tol = 0.001;
          double pre_foot, pre_dot;
          Eigen::Matrix<double, 3, 1> res;

          res = ForwardProp(l2_foot, l1, l1_dot, (prim(4, 0)), aq, eps, backward_num); 
          l2_dot = res(1, 0);
          l2_ddot = 0.002;
          //std::cout << "l2_dot= " << l2_dot << std::endl;
          
          while (n < n_max && std::abs(l2_dot) > max_tol)
          {
            
            pre_foot = l2_foot;
            l2_foot = l2_foot - l2_dot/l2_ddot;
            pre_dot = l2_dot;
            res = ForwardProp(l2_foot, l1, l1_dot, (prim(4, 0)), aq, eps, backward_num); 
            l2_dot = res(1, 0);
            l2_ddot = (l2_dot - pre_dot) / (l2_foot - pre_foot);
            n = n + 1;
            
          }
         
          


          s2 = s2_foot;
          s2_dot = prim(3, 0);

          l2 = res(0, 0);
          l2_dot = res(1, 0);

          v2 = p_foot(2, 0) + prim(2, 0) + prim(4, 0); 
          v2_dot = aq * s2_dot; 

          //std::cout << "l2= " << l2 << std::endl;
          //std::cout << "dy1_n= " << l2 << ", vn= "<< prim(3,0) << std::endl;
          //std::cout << "dy2_n= " << l2_foot << std::endl;
          //std::cout << "used vn= "<< prim(3,0) << std::endl;


          if(stance == 1) 
           {

              if (prim(1, 0) == 0)
              {
          
                        good_vn[sweep_c]=prim(3, 0);
                        //std::cout << "good_vn= "<< good_vn[sweep_c]<< std::endl;
                        sweep_c= sweep_c+1;
                        sum_vn = sum_vn + prim(3,0);

                        new_cost = std::abs((-0.1-l2)) + std::abs((-0.135-(l2_foot - l2)));  //delta_y1 + delta_y2
                        //std::cout << "vn= "<< prim(3, 0) << std::endl;
                        //std::cout << "new_cost= "<< new_cost << std::endl;                         
                        if(new_cost < cost)
                        {
                          cost = new_cost;
                          opt_vn = prim(3, 0);
                          //std::cout << "opt_vn= "<< opt_vn << std::endl;
                        //std::cout << "cost= "<< cost << std::endl;                        
                        }

              }
              else{

                        good_vn[sweep_c]=prim(3, 0);
                        //std::cout << "good_vn= "<< good_vn[sweep_c]<< std::endl;
                        sweep_c= sweep_c+1;
                        sum_vn = sum_vn + prim(3,0);

                        new_cost = 7*std::abs((l2)) + 4*std::abs((-0.135-(l2_foot - l2)));  //delta_y1 + delta_y2
                        //std::cout << "vn= "<< prim(3, 0) << std::endl;
                        //std::cout << "new_cost= "<< new_cost << std::endl;                         
                        if(new_cost < cost)
                        {
                          cost = new_cost;
                          if (prim(3, 0)>= 0.4)
                          {
                              opt_vn = 0.4;
                          }
                          else{
                          opt_vn = prim(3, 0);
                          }
                          //std::cout << "opt_vn= "<< opt_vn << std::endl;
                        //std::cout << "cost= "<< cost << std::endl;                        
                        }

                  }

           }
           else
           {

                if(prim(1, 0)==0)
                {

                        good_vn[sweep_c]=prim(3, 0);
                        //std::cout << "good_vn= "<< good_vn[sweep_c]<< std::endl;
                        sweep_c= sweep_c + 1;
                        sum_vn = sum_vn + prim(3,0);
                        //std::cout << "sweep_c= "<< sweep_c << std::endl;
                        new_cost = std::abs((0.1-l2)) + std::abs((0.135-(l2_foot - l2)));  //delta_y1 + delta_y2
                        //std::cout << "vn= "<< prim(3, 0) << std::endl;
                        //std::cout << "new_cost= "<< new_cost << std::endl;   
                        if(new_cost < cost)
                        {
                          cost = new_cost;
                          opt_vn = prim(3, 0);
                          //std::cout << "opt_vn= "<< opt_vn << std::endl;
                         // std::cout << "cost= "<< cost << std::endl;
                        }

                }
                else
                {
                        good_vn[sweep_c]=prim(3, 0);
                        //std::cout << "good_vn= "<< good_vn[sweep_c]<< std::endl;
                        sweep_c= sweep_c + 1;
                        sum_vn = sum_vn + prim(3,0);
                        //std::cout << "sweep_c= "<< sweep_c << std::endl;
                        new_cost = 7*std::abs((l2)) + 4*std::abs((0.135-(l2_foot - l2)));  //delta_y1 + delta_y2
                        //std::cout << "vn= "<< prim(3, 0) << std::endl;
                        //std::cout << "new_cost= "<< new_cost << std::endl;   
                        if(new_cost < cost)
                        {
                          cost = new_cost;
                            if (prim(3, 0)>= 0.4)
                            {
                                opt_vn = 0.4;
                            }
                            else{
                            opt_vn = prim(3, 0);
                            }
                          //std::cout << "opt_vn= "<< opt_vn << std::endl;
                         // std::cout << "cost= "<< cost << std::endl;
                        }
                }
           }
    

        }
        //std::cout << "dy1_n= " << l2 << ", stance= "<< stance << std::endl ;
        //std::cout <<  "phi= "<< phi << std::endl;
        std::cout <<  "step length= "<< s2-(X_apex(0, 0) - X_d(0, 0))*COS + (X_apex(1, 0) - X_d(1, 0))*SIN << std::endl;
        //std::cout <<  "Bkward Num= "<< backward_num<< std::endl << std::endl << std::endl;

     Eigen::Matrix<double, 8, 1> psp;
     psp << (X_apex(0, 0) - X_d(0, 0))*COS + (X_apex(1, 0) - X_d(1, 0))*SIN, s1_foot, X_apex(4, 0)*std::cos(prim(1, 0)-phi), s2, s2_foot, s2_dot, eps*forward_num, eps*backward_num;
     psp_list.push_back(psp);
    //goodlateral = 1;
    // p_foot update (next step)
    p_foot(0, 0) = s2_foot*COS - l2_foot*SIN + X_d(0, 0);
    p_foot(1, 0) = s2_foot*SIN + l2_foot*COS + X_d(1, 0);
    p_foot(2, 0) = p_foot(2, 0) + prim(2, 0); 
    
    v2_foot = p_foot(2, 0);
    // X_switch
    X_switch(0, 0) = s_switch*COS - l_switch*SIN + X_d(0, 0);
    X_switch(1, 0) = s_switch*SIN + l_switch*COS + X_d(1, 0);
    X_switch(2, 0) = v_switch;
    X_switch(3, 0) = ds_switch*COS - dl_switch*SIN;
    X_switch(4, 0) = ds_switch*SIN + dl_switch*COS;
    X_switch(5, 0) = dv_switch;
    
    // apex trajectory keyframe1
    apextraj(0, 0)= X_apex(0, 0); //xapex
    apextraj(1, 0)= X_apex(1, 0);//yapex
    apextraj(2, 0)= X_apex(2, 0);  //zapex
    apextraj(3, 0)= X_apex(4, 0)*std::cos(X_apex(3, 0)+phi); //xd
    apextraj(4, 0)= X_apex(4, 0)*std::sin(X_apex(3, 0)+phi); //yd
    apextraj(5, 0)= aq*apextraj(3, 0); //zd

        
    // Keyframe 
    X_apex(0, 0) = s2*COS - l2*SIN + X_d(0, 0);
    X_apex(1, 0) = s2*SIN + l2*COS + X_d(1, 0);
    X_apex(2, 0) = p_foot(2, 0) + prim(4, 0); 
    X_apex(3, 0) += prim(1, 0) ; //X_d(3, 0) + prim(1, 0) + phi;
    X_apex(4, 0) = prim(3, 0);
    

    //Eigen::Matrix<double, 5, 1> pre_apex = apex_list[step-1];
    //double prev_heading = pre_apex(3, 0);
    //std::cout << "current= " << X_apex(3,0) << "  prev= " << prev_heading << std::endl << std::endl << std::endl;
      if (prim(0, 0) > 0.4 && prim(0, 0) < 0.46)
      {
        sag = 4;
      }
      else if (prim(0, 0) > 0.46)
      {
        sag = 5;
      }
      else if (prim(0, 0) > 0.3 && prim(0, 0) < 0.4)
      {
        sag = 3;
      }
      else if (prim(0, 0) > 0.2 && prim(0, 0) < 0.3)
      {
        sag = 3;
      }


    if ((prim(1, 0) != 0) && ( (std::cos(X_apex(3,0)) > 0.95) || (std::cos(X_apex(3,0)) < -0.95) || (std::sin(X_apex(3,0))>0.95) || (std::sin(X_apex(3,0)) < -0.95)  )) //((prim(1, 0) == 0) && (X_apex(3,0) != prev_heading))
    {

       // std::cout << "**here**" << std::endl << std::endl;

    /*  // nominal next fine cell number difference     
      if(std::sin(X_apex(3, 0)) > 0.8 )
      {
        fine_num = 4;
      }
      else if(std::sin(X_apex(3, 0)) < -0.8)
      {
        fine_num = -4;
      }
      else if(std::cos(X_apex(3, 0)) < -0.8 )
      {
        fine_num = -4*26;
      }     
      else if(std::cos(X_apex(3, 0)) > 0.8)
      {
        fine_num = 4*26;
      }      
    */
      if (stance == 0) //fine cell adjusment 
      {
          if (l2 > 0.314)
        {
           X_d(0, 0) += prim(0, 0)*std::cos(X_apex(3, 0)) -0.208*std::sin(X_apex(3, 0)); //X_apex(0, 0);
           X_d(1, 0) += prim(0, 0)*std::sin(X_apex(3, 0)) +0.208*std::cos(X_apex(3, 0)); //X_apex(1, 0);
           X_d(2, 0) =  p_foot(2, 0) + prim(4, 0); 
           X_d(3, 0) += prim(1, 0);
           X_d(4, 0) = prim(3, 0);
            
           //fine cell adjustment 
              if(std::sin(X_apex(3, 0)) > 0.8 )
              {
                fine_num = -26;
                N=2; S=0; E=sag; W=0;
              }
              else if(std::sin(X_apex(3, 0)) < -0.8)
              {
                fine_num = 26;
                N=0; S=2; E=0; W=sag;
              }
              else if(std::cos(X_apex(3, 0)) < -0.8 )
              {
                fine_num = -1;
                N=sag; S=0; E=0; W=2;
              }     
              else if(std::cos(X_apex(3, 0)) > 0.8)
              {
                fine_num = 1;
                N=0; S=sag; E=2; W=0;
              } 

         }
          else if (l2 > 0.2)
        {
           X_d(0, 0) += prim(0, 0)*std::cos(X_apex(3, 0)) -0.104*std::sin(X_apex(3, 0)); //X_apex(0, 0);
           X_d(1, 0) += prim(0, 0)*std::sin(X_apex(3, 0)) +0.104*std::cos(X_apex(3, 0)); //X_apex(1, 0);
           X_d(2, 0) =  p_foot(2, 0) + prim(4, 0); 
           X_d(3, 0) += prim(1, 0);
           X_d(4, 0) = prim(3, 0);
            
           //fine cell adjustment 
              if(std::sin(X_apex(3, 0)) > 0.8 )
              {
                fine_num = -26;
                N=1; S=0; E=sag; W=0;
              }
              else if(std::sin(X_apex(3, 0)) < -0.8)
              {
                fine_num = 26;
                N=0; S=1; E=0; W=sag;
              }
              else if(std::cos(X_apex(3, 0)) < -0.8 )
              {
                fine_num = -1;
                N=sag; S=0; E=0; W=1;
              }     
              else if(std::cos(X_apex(3, 0)) > 0.8)
              {
                fine_num = 1;
                N=0; S=sag; E=1; W=0;
              } 

         }
         
         else if (l2 < 0)
         {
        //  if (l2 + 0.104)
        //  {
           X_d(0, 0) += prim(0, 0)*std::cos(X_apex(3, 0)) +0.104*std::sin(X_apex(3, 0)); //X_apex(0, 0);
           X_d(1, 0) += prim(0, 0)*std::sin(X_apex(3, 0)) -0.104*std::cos(X_apex(3, 0)); //X_apex(1, 0);
           X_d(2, 0) =  p_foot(2, 0) + prim(4, 0); 
           X_d(3, 0) += prim(1, 0);
           X_d(4, 0) = prim(3, 0);
        //  }
        //  else
         // {
       //    X_d(0, 0) += prim(0, 0)*std::cos(X_apex(3, 0)) +0.208*std::sin(X_apex(3, 0)); //X_apex(0, 0);
        //   X_d(1, 0) += prim(0, 0)*std::sin(X_apex(3, 0)) -0.208*std::cos(X_apex(3, 0)); //X_apex(1, 0);
        //   X_d(2, 0) =  p_foot(2, 0) + prim(4, 0); 
        //   X_d(3, 0) += prim(1, 0);
        //   X_d(4, 0) = prim(3, 0);

        //  }
           //X_d(0, 0) += prim(0, 0)*std::cos(X_apex(3, 0)) +0.104*std::sin(X_apex(3, 0)); //X_apex(0, 0);
          // X_d(1, 0) += prim(0, 0)*std::sin(X_apex(3, 0)) -0.104*std::cos(X_apex(3, 0)); //X_apex(1, 0);
          // X_d(2, 0) =  p_foot(2, 0) + prim(4, 0); 
          // X_d(3, 0) += prim(1, 0);
          // X_d(4, 0) = prim(3, 0);
            //std::cout <<  "here 11 " << std::endl;

              if(std::sin(X_apex(3, 0)) > 0.8 )
              {
                fine_num = 26;
                N=0; S=1; E=sag; W=0;
              }
              else if(std::sin(X_apex(3, 0)) < -0.8)
              {
                fine_num = -26;
                N=1; S=0; E=0; W=sag;
              }
              else if(std::cos(X_apex(3, 0)) < -0.8 )
              {
                fine_num = 1;
                N=sag; S=0; E=1; W=0;
              }     
              else if(std::cos(X_apex(3, 0)) > 0.8)
              {
                fine_num = -1;
                N=0; S=sag; E=0; W=1;
              } 
         }
         
           else{
            X_d(0, 0) += prim(0, 0)*std::cos(X_apex(3, 0)); //X_apex(0, 0);
            X_d(1, 0) += prim(0, 0)*std::sin(X_apex(3, 0)); //X_apex(1, 0);
            X_d(2, 0) =  p_foot(2, 0) + prim(4, 0); 
            X_d(3, 0) += prim(1, 0);
            X_d(4, 0) = prim(3, 0);  
                    if(std::sin(X_apex(3, 0)) > 0.8 )
                    {
                       N=0;  S=0;  E=sag;  W=0;          
                    }
                    else if(std::sin(X_apex(3, 0)) < -0.8)
                    {
                       N=0;  S=0;  E=0;  W=sag;  
                    }
                    else if(std::cos(X_apex(3, 0)) < -0.8 )
                    {
                      N=sag;  S=0;  E=0;  W=0;  
                    }     
                    else if(std::cos(X_apex(3, 0)) > 0.8)
                    {
                      N=0;  S=sag;  E=0;  W=0;  
                    }      

           }
      }
      else
      {
          if (l2 < -0.314)
        {
          X_d(0, 0) += prim(0, 0)*std::cos(X_apex(3, 0)) +0.208*std::sin(X_apex(3, 0)); //X_apex(0, 0);
          X_d(1, 0) += prim(0, 0)*std::sin(X_apex(3, 0)) -0.208*std::cos(X_apex(3, 0)); //X_apex(1, 0);
          X_d(2, 0) =  p_foot(2, 0) + prim(4, 0); 
          X_d(3, 0) += prim(1, 0);
          X_d(4, 0) = prim(3, 0);
             // std::cout <<  "here 2 " << std::endl;
              if(std::sin(X_apex(3, 0)) > 0.8 )
              {
                fine_num = 26;
                N=0; S=2; E=sag; W=0;
              }
              else if(std::sin(X_apex(3, 0)) < -0.8)
              {
                fine_num = -26;
                N=2; S=0; E=0; W=sag;                
              }
              else if(std::cos(X_apex(3, 0)) < -0.8 )
              {
                fine_num = 1;
                N=sag; S=0; E=2; W=0;                
              }     
              else if(std::cos(X_apex(3, 0)) > 0.8)
              {
                fine_num = -1;
                N=0; S=sag; E=0; W=2;               
              } 



        }

         else if (l2 < -0.2)
        {
          X_d(0, 0) += prim(0, 0)*std::cos(X_apex(3, 0)) +0.104*std::sin(X_apex(3, 0)); //X_apex(0, 0);
          X_d(1, 0) += prim(0, 0)*std::sin(X_apex(3, 0)) -0.104*std::cos(X_apex(3, 0)); //X_apex(1, 0);
          X_d(2, 0) =  p_foot(2, 0) + prim(4, 0); 
          X_d(3, 0) += prim(1, 0);
          X_d(4, 0) = prim(3, 0);
           //   std::cout <<  "here 2 " << std::endl;
              if(std::sin(X_apex(3, 0)) > 0.8 )
              {
                fine_num = 26;
                N=0; S=1; E=sag; W=0;
              }
              else if(std::sin(X_apex(3, 0)) < -0.8)
              {
                fine_num = -26;
                N=1; S=0; E=0; W=sag;                
              }
              else if(std::cos(X_apex(3, 0)) < -0.8 )
              {
                fine_num = 1;
                N=sag; S=0; E=1; W=0;                
              }     
              else if(std::cos(X_apex(3, 0)) > 0.8)
              {
                fine_num = -1;
                N=0; S=sag; E=0; W=1;               
              } 



        }
        else if (l2 > 0)
        {
          X_d(0, 0) += prim(0, 0)*std::cos(X_apex(3, 0)) -0.104*std::sin(X_apex(3, 0)); //X_apex(0, 0);
          X_d(1, 0) += prim(0, 0)*std::sin(X_apex(3, 0)) +0.104*std::cos(X_apex(3, 0)); //X_apex(1, 0);
          X_d(2, 0) =  p_foot(2, 0) + prim(4, 0); 
          X_d(3, 0) += prim(1, 0);
          X_d(4, 0) = prim(3, 0);
             // std::cout <<  "here 22 " << std::endl;
              if(std::sin(X_apex(3, 0)) > 0.8 )
              {
                fine_num = -26;
                N=1; S=0; E=sag; W=0; 
              }
              else if(std::sin(X_apex(3, 0)) < -0.8)
              {
                fine_num = 26;
                N=0; S=1; E=0; W=sag; 
              }
              else if(std::cos(X_apex(3, 0)) < -0.8 )
              {
                fine_num = -1;
                N=sag; S=0; E=0; W=1; 
              }     
              else if(std::cos(X_apex(3, 0)) > 0.8)
              {
                fine_num = 1;
                N=0; S=sag; E=1; W=0;            
              } 
        }
        
        else{
          X_d(0, 0) += prim(0, 0)*std::cos(X_apex(3, 0)); //X_apex(0, 0);
          X_d(1, 0) += prim(0, 0)*std::sin(X_apex(3, 0)); //X_apex(1, 0);
          X_d(2, 0) =  p_foot(2, 0) + prim(4, 0); 
          X_d(3, 0) += prim(1, 0);
          X_d(4, 0) = prim(3, 0);
                    if(std::sin(X_apex(3, 0)) > 0.8 )
                    {
                       N=0;  S=0;  E=sag;  W=0;          
                    }
                    else if(std::sin(X_apex(3, 0)) < -0.8)
                    {
                      fine_num = -4;
                       N=0;  S=0;  E=0;  W=sag;  
                    }
                    else if(std::cos(X_apex(3, 0)) < -0.8 )
                    {
                      fine_num = -4*26;
                      N=sag;  S=0;  E=0;  W=0;  
                    }     
                    else if(std::cos(X_apex(3, 0)) > 0.8)
                    {
                      fine_num = 4*26;
                      N=0;  S=sag;  E=0;  W=0;  
                    }      
        }
      }
    } 
    else
    {
      X_d(0, 0) += prim(0, 0)*std::cos(X_apex(3, 0)); //X_apex(0, 0);
      X_d(1, 0) += prim(0, 0)*std::sin(X_apex(3, 0)); //X_apex(1, 0);
      X_d(2, 0) =  p_foot(2, 0) + prim(4, 0); 
      X_d(3, 0) += prim(1, 0);
      X_d(4, 0) = prim(3, 0);
                    if(std::sin(X_apex(3, 0)) > 0.8 )
                    {
                       N=0;  S=0;  E=sag;  W=0;          
                    }
                    else if(std::sin(X_apex(3, 0)) < -0.8)
                    {
                      fine_num = -4;
                       N=0;  S=0;  E=0;  W=sag;  
                    }
                    else if(std::cos(X_apex(3, 0)) < -0.8 )
                    {
                      fine_num = -4*26;
                      N=sag;  S=0;  E=0;  W=0;  
                    }     
                    else if(std::cos(X_apex(3, 0)) > 0.8)
                    {
                      fine_num = 4*26;
                      N=0;  S=sag;  E=0;  W=0;  
                    }      
    }


    //std::cout << "stance= " << stance << "N= " << N << "S= " << S << "E= " << E << "W= " << W <<  std::endl << std::endl;

    waypoint(0, 0) += prim(0, 0)*std::cos(X_apex(3, 0));
    waypoint(1, 0) += prim(0, 0)*std::sin(X_apex(3, 0));
    waypoint(2, 0) =  p_foot(2, 0) + prim(4, 0); 
    waypoint(3, 0) += prim(1, 0);
    waypoint(4, 0) = prim(3, 0);

 
    // Record
    step += 1;
    if (stance == 0)
    {
      stance = 1;
    }
    else
    {
      stance = 0;
    }

      



    foot_dis = l2_foot - l2;
    // sim_list 
    Eigen::Matrix<double, 3, 1> sim;
    sim << l2, foot_dis, prim(1,0);
    sim_list.push_back(sim);
    //step lest
    Eigen::Matrix<double, 2, 1> step_sim;
    step_sim << std::abs(s2_foot-s1_foot),std::abs(l2_foot-l1_foot);
    step_list.push_back(step_sim);

    apex_list.push_back(X_apex);
    d_list.push_back(X_d);
    switch_list.push_back(X_switch);

    apextraj_list.push_back(apextraj);

    waypoint_list.push_back(waypoint);

    p_foot_list.push_back(p_foot);

    Eigen::Matrix<double, 3, 1> direction;
    direction << X_apex(3,0) - prim(1, 0),
                 X_apex(3,0) - prim(1, 0)*backward_num/(forward_num+backward_num),
                 X_apex(3,0);
    direction_list.push_back(direction);

    Eigen::Matrix<double, 2, 1> dt;
    dt << eps*forward_num, eps*backward_num;
    step_period.push_back(dt);
    Eigen::Matrix<double, 2, 1> slope;
    slope << aq * std::cos(X_apex(3, 0)), aq * std::sin(X_apex(3, 0));
    //slope << aq , bq ;
    step_surface.push_back(slope);

    Eigen::Matrix<double, 2, 1> WSQ;
    WSQ << w1_sq, w2_sq;
    WSQlist.push_back(WSQ);

        // step time, t1, t2
      Eigen::Matrix<double, 3, 1> time;
      time <<  eps*(forward_num+backward_num), eps*forward_num, eps*backward_num;
      Tlist.push_back(time);
    
  }

  void PhaseSpacePlanner::UpdateTrajectory() 
  {
    double T = step_period[step-2](1, 0) + step_period[step-1](0, 0);
    double dt = 0.001; 
    int num = T/dt;
    double tt1=step_period[step-1](0, 0);
    double tt2=step_period[step-1](1, 0);
    Eigen::Matrix<double, 1, 1> TS;
    Tstep = Tstep + T;
    TS << Tstep;
    double tt3=step_period[step-2](1, 0);
    //Tlist.push_back(TS);
    //std::cout << "num= " << num << std::endl;

 

    Eigen::Matrix<double, 6, 1> start = apextraj_list[step-1];//apextraj_list[step-1];//switch_list[step-2];
    Eigen::Matrix<double, 6, 1> end = switch_list[step-1];//apextraj_list[step];//switch_list[step-1];

    Eigen::Matrix<double, 3, 1> pre_foot = p_foot_list[step-2];
    Eigen::Matrix<double, 3, 1> cur_foot = p_foot_list[step-1];
    Eigen::Matrix<double, 3, 1> nex_foot = p_foot_list[step];
    Eigen::Matrix<double, 3, 1> nex2_foot = p_foot_list[step];

    Eigen::Matrix<double, 2, 1> slope1 = step_surface[step-1];
    Eigen::Matrix<double, 2, 1> slope2 = step_surface[step-1];

    Eigen::Matrix<double, 2, 1> wwsq = WSQlist[step-3];

    double x_ddot, x_dot = start(3, 0), x = start(0, 0);
    double y_ddot, y_dot = start(4, 0), y = start(1, 0);
    double z_ddot, z_dot = start(5, 0), z = start(2, 0);

    Eigen::Matrix<double, 10, 1> swing_init_foot;
    if(stance == 1)
    {
        swing_init_foot=l_foot_list.back();
    }
    else
    {
        swing_init_foot=r_foot_list.back();
    }

    double x_swing0 = swing_init_foot(0,0) , dx_swing, ddx_swing;
    double y_swing0 = swing_init_foot(1,0), dy_swing, ddy_swing;
    double z_swing0 = swing_init_foot(2,0), dz_swing, ddz_swing;
    double x_swing;
    double y_swing;
    double z_swing;
    //z_swing0=0;
    //x_swing0=0;
    //y_swing0=0;
    double x_distance = nex_foot(0, 0) - pre_foot(0, 0); 
    double y_distance = nex_foot(1, 0) - pre_foot(1, 0);
    //std::cout << y_distance << "   " << x_distance << std::endl;
    double dh = 0.3;
    double h1 = dh;
    double h2 = dh + pre_foot(2, 0) - nex_foot(2, 0);
    double t1, t2, ts;
    double vmax = 1.5;
    bool steady = false;

    if (h1 > h2)
    {
      t1 = h1 * PI / (2 * vmax);
      t2 = t1 * h2 / h1;
    }
    else
    {
      t2 = h2 * PI / (2 * vmax);
      t1 = t2 * h1 / h2;
    }
    
    if (T < t1 + t2)//(T < t1 + t2)
    {
      h1 = (T * h1 / (h1 + h2)) * 2 * vmax / PI;
      h2 = h1 + pre_foot(2, 0) - nex_foot(2, 0);
      t1 = T * h1 / (h1 + h2);
      t2 = T * h2 / (h1 + h2);
      ts = 0;
    }
    else
    {
      steady = true;
      ts = T-t1-t2;//T - t1 - t2;
    }

    int num0 = step_period[step-2](1, 0) / dt;
    //int num1 = t1 / dt;
    //int num2 = t2 / dt;
    //int num3 = ts / dt;
    int num4 = tt3/dt;


    int numtt= (tt1+tt2)/dt;
    int num11= (tt1)/dt;
    int num22= tt2/dt;

    double w_sq = wwsq(1, 0);
    
    double inc_x, inc_y;
    Eigen::Matrix<double, 10, 1> stance_foot;

    for (int i = 0; i < numtt; i++)//(int i = 0; i < num; i++)
    { 
    /*  
      //if(x < end(0, 0))
      //{
        x_ddot = w_sq * (x - cur_foot(0, 0));
        inc_x = dt * x_dot + 0.5 * dt * dt * x_ddot;
        x = x + inc_x;
        x_dot = x_dot + dt * x_ddot;

        y_ddot = w_sq * (y - cur_foot(1, 0));
        inc_y = dt * y_dot + 0.5 * dt * dt * y_ddot;
        y = y + inc_y;
        y_dot = y_dot + dt * y_ddot;
  */  //  }
      if (i < num11)
      { 
        
       x_ddot = w_sq * (x - cur_foot(0, 0));
       inc_x = dt * x_dot + 0.5 * dt * dt * x_ddot;
       x = x + inc_x;
       x_dot = x_dot + dt * x_ddot;

       y_ddot = w_sq * (y - cur_foot(1, 0));
       inc_y = dt * y_dot + 0.5 * dt * dt * y_ddot;
       y = y + inc_y;
       y_dot = y_dot + dt * y_ddot;


        z_ddot = slope1(0, 0) * x_ddot + slope1(1, 0) * y_ddot;
        z = z + slope1(0, 0) * inc_x + slope1(1, 0) * inc_y;
        z_dot = slope1(0, 0) * x_dot + slope1(1, 0) * y_dot;

        Eigen::Matrix<double, 1, 1> heading;
        heading << (direction_list[step-2](2, 0) - direction_list[step-2](1, 0))
                  *(i + 1)/num0 
                  + direction_list[step-2](1, 0);
        heading_list.push_back(heading);

        //x_swing = pre_foot(0, 0) + x_distance / 2 * (1 - std::cos(PI * (i+num4) / numtt));
        x_swing = nex_foot(0, 0) + (x_swing0 - nex_foot(0, 0)) * (std::cos(0.5*PI * (i+num4) / numtt));
        dx_swing = x_distance / 2 * std::sin(PI * i / numtt) * PI / T;
        ddx_swing = x_distance / 2 * std::cos(PI * i / numtt) * std::pow(PI / T, 2);

        //y_swing = pre_foot(1, 0) + y_distance / 2 * (1 - std::cos(PI * (i+num4) / numtt));
        y_swing = nex_foot(1, 0) + (y_swing0 - nex_foot(1, 0)) * (std::cos(0.5*PI * (i+num4) / numtt));
        dy_swing = y_distance / 2 * std::sin(PI * i / numtt) * PI / T;
        ddy_swing = y_distance / 2 * std::cos(PI * i / numtt) * std::pow(PI / T, 2);

        //Eigen::Matrix<double, 10, 1> stance_foot;
        stance_foot << cur_foot(0, 0), cur_foot(1, 0), cur_foot(2, 0), 0, 0, 0, 0, 0, 0, d_t;
      }
      else
      { 

          x_ddot = w_sq * (x - nex_foot(0, 0));
          inc_x = dt * x_dot + 0.5 * dt * dt * x_ddot;
          x = x + inc_x;
          x_dot = x_dot + dt * x_ddot;

          y_ddot = w_sq * (y - nex_foot(1, 0));
          inc_y = dt * y_dot + 0.5 * dt * dt * y_ddot;
          y = y + inc_y;
          y_dot = y_dot + dt * y_ddot;

        z_ddot = slope2(0, 0) * x_ddot + slope2(1, 0) * y_ddot;
        z = z + slope2(0, 0) * inc_x + slope2(1, 0) * inc_y;
        z_dot = slope2(0, 0) * x_dot + slope2(1, 0) * y_dot;

        Eigen::Matrix<double, 1, 1> heading;
        heading << (direction_list[step-1](1, 0) - direction_list[step-1](0, 0))
                  *(i  + 1 - num0)/(num - num0)
                  + direction_list[step-1](0, 0);
        heading_list.push_back(heading);
        
        x_swing = cur_foot(0, 0) + x_distance / 4 * (1 - std::cos(PI * (i-num11) / num22));
        dx_swing = x_distance / 4 * std::sin(PI * (i-num11)/ num22) * PI / T;
        ddx_swing = x_distance / 4 * std::cos(PI * (i-num11) / num22) * std::pow(PI / T, 2);

        y_swing = cur_foot(1, 0) + y_distance / 4 * (1 - std::cos(PI * (i-num11)/ num22));
        dy_swing = y_distance / 4 * std::sin(PI * (i-num11) / num22) * PI / T;
        ddy_swing = y_distance / 4 * std::cos(PI * (i-num11) / num22) * std::pow(PI / T, 2);

      //Eigen::Matrix<double, 10, 1> stance_foot;
        stance_foot << nex_foot(0, 0), nex_foot(1, 0), nex_foot(2, 0), 0, 0, 0, 0, 0, 0, d_t;
      }

      Eigen::Matrix<double, 10, 1> COM;
      COM << x, y, z, x_dot, y_dot, z_dot, x_ddot, y_ddot, z_ddot, d_t;
      COM_list.push_back(COM);

      //x_swing = cur_foot(0, 0) + x_distance / 2 * (1 - std::cos(PI * i / num));
      //dx_swing = x_distance / 2 * std::sin(PI * i / num) * PI / T;
      //ddx_swing = x_distance / 2 * std::cos(PI * i / num) * std::pow(PI / T, 2);

     // y_swing = cur_foot(1, 0) + y_distance / 2 * (1 - std::cos(PI * i / num));
     // dy_swing = y_distance / 2 * std::sin(PI * i / num) * PI / T;
     // ddy_swing = y_distance / 2 * std::cos(PI * i / num) * std::pow(PI / T, 2);


      if(i < num11)
      {
        z_swing =nex_foot(2,0)  + (z_swing0 - nex_foot(2,0)) * (std::cos(0.5 * PI * (i) / num11));
        //std::cout <<  (std::cos(PI * (i) / numtt)) << std::endl;
        dz_swing = -h1 / 2 * std::sin(PI * i / num11) * PI / tt1;
        ddz_swing = -h1 / 2 * std::cos(PI * i / num11) * std::pow(PI / tt1, 2);
        //std::cout << z_swing << std::endl;
      }
     // else if (i < num1 + num3)
      //{
       // z_swing = pre_foot(2, 0) + h1;
      // // dz_swing = 0;
      //  ddz_swing = 0;
     // }  
      else
      {
        z_swing = cur_foot(2, 0) + h2/2 * ( std::sin(0.5* PI * (i-num11) / num22));
        
        dz_swing = h2 / 2 * std::sin(PI * (i-num11) / num22) * PI / t2;
        ddz_swing = h2 / 2 * std::cos(PI * (i-num11) / num22) * std::pow(PI / t2, 2);
      }

      Eigen::Matrix<double, 10, 1> swing_foot;
      swing_foot << x_swing, y_swing, z_swing, dx_swing, dy_swing, dz_swing, 
                    ddx_swing, ddy_swing, ddz_swing, d_t;
    if(i < num11)
    {
      if (stance == 0)
      {
        r_foot_list.push_back(swing_foot);
        l_foot_list.push_back(stance_foot);
      }
      else
      {
        l_foot_list.push_back(swing_foot);
        r_foot_list.push_back(stance_foot);
      }
    }
    else
    {
      if (stance == 1)
      {
        r_foot_list.push_back(swing_foot);
        l_foot_list.push_back(stance_foot);
      }
      else
      {
        l_foot_list.push_back(swing_foot);
        r_foot_list.push_back(stance_foot);
      }

    }
      d_t += 0.001;
    
    }
  }

  void PhaseSpacePlanner::Start(int stance_start)
  { 

    if (stance_start == 1)
    {
      stance = 1;

    }
    else
    {
      stance = 0;
      
     
    }

    
  }

  void PhaseSpacePlanner::FirstStep()
  { 

   
    double T = step_period[step-1](0, 0);
    double dt = 0.001;
    int num = T/dt;
    double tt1=step_period[step-1](0, 0);
    double tt2=step_period[step-1](1, 0);
    double tt3=step_period[step-2](1, 0);

    Eigen::Matrix<double, 1, 1> TS;
    Tstep = Tstep + T;
    TS << Tstep;
    //Tlist.push_back(TS);
    
    //Eigen::Matrix<double, 6, 1> start;
    /*
    start << apex_list[step-1](0, 0), apex_list[step-1](1, 0), apex_list[step-1](2, 0), 
             apex_list[step-1](4, 0)*cos(apex_list[step-1](3, 0)), 
             apex_list[step-1](4, 0)*sin(apex_list[step-1](3, 0)), 0;
     */        
    Eigen::Matrix<double, 6, 1> start = apextraj_list[step-1];
    Eigen::Matrix<double, 6, 1> end = switch_list[step-1];
    Eigen::Matrix<double, 2, 1> wwsq = WSQlist[step-2];
    Eigen::Matrix<double, 3, 1> pre_foot;

    if (stance == 1)
    {
      pre_foot << apex_list[step-1](0, 0) + 0.135 * std::sin(-apex_list[step-1](3, 0)),
                  apex_list[step-1](1, 0) + 0.135 * std::cos(-apex_list[step-1](3, 0)), 
                  p_foot_list[step-1](2, 0);      
    }
    else
    {
      pre_foot << apex_list[step-1](0, 0) - 0.135 * std::sin(-apex_list[step-1](3, 0)),
                  apex_list[step-1](1, 0) - 0.135 * std::cos(-apex_list[step-1](3, 0)), 
                  p_foot_list[step-1](2, 0);
    }
    Eigen::Matrix<double, 3, 1> cur_foot = p_foot_list[step-1];
    Eigen::Matrix<double, 3, 1> nex_foot = p_foot_list[step];

    Eigen::Matrix<double, 2, 1> slope = step_surface[step-1];
    double x_ddot, x_dot = start(3, 0), x = start(0, 0);
    double y_ddot, y_dot = start(4, 0), y = start(1, 0);
    double z_ddot, z_dot = start(5, 0), z = start(2, 0);
/*
    double x_swing, dx_swing, ddx_swing;
    double y_swing, dy_swing, ddy_swing;
    double z_swing, dz_swing, ddz_swing;
*/  

    Eigen::Matrix<double, 10, 1> swing_init_foot;
    if(stance == 1)
    {
        swing_init_foot=l_foot_list.back();
    }
    else
    {
        swing_init_foot=r_foot_list.back();
    }

    double x_swing0 = swing_init_foot(0,0) , dx_swing, ddx_swing;
    double y_swing0 = swing_init_foot(1,0), dy_swing, ddy_swing;
    double z_swing0 = swing_init_foot(2,0), dz_swing, ddz_swing;
    double x_swing;
    double y_swing;
    double z_swing;

    //y_swing0=0;
    //x_swing0=0;
    double x_distance = nex_foot(0, 0) - pre_foot(0, 0); 
    double y_distance = nex_foot(1, 0) - pre_foot(1, 0);
    double dh = 0.3;
    double h1 = dh;
    double h2 = dh + pre_foot(2, 0) - nex_foot(2, 0);
    double t1, t2, ts;
    double vmax = 1.5;
    bool steady = false;

    if (h1 > h2)
    {
      t1 = h1 * PI / (2 * vmax);
      t2 = t1 * h2 / h1;
    }
    else
    {
      t2 = h2 * PI / (2 * vmax);
      t1 = t2 * h1 / h2;
    }
    
    if (T < t1 + t2)
    {
      h1 = (T * h1 / (h1 + h2)) * 2 * vmax / PI;
      h2 = h1 + pre_foot(2, 0) - nex_foot(2, 0);
      t1 = T * h1 / (h1 + h2);
      t2 = T * h2 / (h1 + h2);
      ts = 0;
    }
    else
    {
      steady = true;
      ts = T - t1 - t2;
    }

    //int num1 = t1 / dt;
   // int num2 = t2 / dt;
    //int num3 = ts / dt;

    double w_sq = wwsq(1, 0); 
    
    double inc_x, inc_y;
    int numtt= (tt1+tt2)/dt;
    int num11= (tt1)/dt;
    int num22= tt2/dt;

    int num4 = tt3/dt;


    Eigen::Matrix<double, 10, 1> stance_foot;

    // step time, t1, t2
    /*
      Eigen::Matrix<double, 3, 1> time;
      time << TS, t1, t2;
      Tlist.push_back(time);
      */
    for (int i = 0; i < numtt; i++)
    {
      d_t += 0.001;
      if( i < num11)
      {
        x_ddot = w_sq * (x - cur_foot(0, 0));
        inc_x = dt * x_dot + 0.5 * dt * dt * x_ddot;
        x = x + inc_x;
        x_dot = x_dot + dt * x_ddot;

        y_ddot = w_sq * (y - cur_foot(1, 0));
        inc_y = dt * y_dot + 0.5 * dt * dt * y_ddot;
        y = y + inc_y;
        y_dot = y_dot + dt * y_ddot;

        z_ddot = slope(0, 0) * x_ddot + slope(1, 0) * y_ddot;
        z = z + slope(0, 0) * inc_x + slope(1, 0) * inc_y;
        z_dot = slope(0, 0) * x_dot + slope(1, 0) * y_dot;
              Eigen::Matrix<double, 1, 1> heading;
      heading << (direction_list[step-1](1, 0) - direction_list[step-1](0, 0))
                  *(i  + 1)/num + direction_list[step-1](0, 0);
      heading_list.push_back(heading);
/*
       y_swing = pre_foot(1, 0) + y_distance / 2 * (1 - std::cos(PI * i / num11));
       dy_swing = y_distance / 2 * std::sin(PI * i / num11) * PI / T;
       ddy_swing = y_distance / 2 * std::cos(PI * i / num11) * std::pow(PI / T, 2);
       x_swing = pre_foot(0, 0) + x_distance / 2 * (1 - std::cos(PI * i / num11));
       dx_swing = x_distance / 2 * std::sin(PI * i / num11) * PI / T;
       ddx_swing = x_distance / 2 * std::cos(PI * i / num11) * std::pow(PI / T, 2);
*/

        x_swing = nex_foot(0, 0) + (x_swing0 - nex_foot(0, 0)) * (std::cos(0.5*PI * (i+num4) / numtt));
        dx_swing = x_distance / 2 * std::sin(PI * i / numtt) * PI / T;
        ddx_swing = x_distance / 2 * std::cos(PI * i / numtt) * std::pow(PI / T, 2);

        //y_swing = pre_foot(1, 0) + y_distance / 2 * (1 - std::cos(PI * (i+num4) / numtt));
        y_swing = nex_foot(1, 0) + (y_swing0 - nex_foot(1, 0)) * (std::cos(0.5*PI * (i+num4) / numtt));
        dy_swing = y_distance / 2 * std::sin(PI * i / numtt) * PI / T;
        ddy_swing = y_distance / 2 * std::cos(PI * i / numtt) * std::pow(PI / T, 2);

        //Eigen::Matrix<double, 10, 1> stance_foot;
        stance_foot << cur_foot(0, 0), cur_foot(1, 0), cur_foot(2, 0), 0, 0, 0, 0, 0, 0, d_t;
      }
      
      else
      { 

          x_ddot = w_sq * (x - nex_foot(0, 0));
          inc_x = dt * x_dot + 0.5 * dt * dt * x_ddot;
          x = x + inc_x;
          x_dot = x_dot + dt * x_ddot;

          y_ddot = w_sq * (y - nex_foot(1, 0));
          inc_y = dt * y_dot + 0.5 * dt * dt * y_ddot;
          y = y + inc_y;
          y_dot = y_dot + dt * y_ddot;

        z_ddot = slope(0, 0) * x_ddot + slope(1, 0) * y_ddot;
        z = z + slope(0, 0) * inc_x + slope(1, 0) * inc_y;
        z_dot = slope(0, 0) * x_dot + slope(1, 0) * y_dot;

                   Eigen::Matrix<double, 1, 1> heading;
      heading << (direction_list[step-1](1, 0) - direction_list[step-1](0, 0))
                  *(i  + 1)/num + direction_list[step-1](0, 0);
      heading_list.push_back(heading);

        x_swing = cur_foot(0, 0) + x_distance / 4 * (1 - std::cos(PI * (i-num11) / num22));
        dx_swing = x_distance / 4 * std::sin(PI * (i-num11)/ num22) * PI / T;
        ddx_swing = x_distance / 4 * std::cos(PI * (i-num11) / num22) * std::pow(PI / T, 2);

        y_swing = cur_foot(1, 0) + y_distance / 4 * (1 - std::cos(PI * (i-num11)/ num22));
        dy_swing = y_distance / 4 * std::sin(PI * (i-num11) / num22) * PI / T;
        ddy_swing = y_distance / 4 * std::cos(PI * (i-num11) / num22) * std::pow(PI / T, 2);

      //Eigen::Matrix<double, 10, 1> stance_foot;
        stance_foot << nex_foot(0, 0), nex_foot(1, 0), nex_foot(2, 0), 0, 0, 0, 0, 0, 0, d_t;
      }


      Eigen::Matrix<double, 10, 1> COM;
      COM << x, y, z, x_dot, y_dot, z_dot, x_ddot, y_ddot, z_ddot, d_t;
      COM_list.push_back(COM);



    



      if(i < num11)
      {
        z_swing =nex_foot(2,0)  + (h2/2 + z_swing0 - nex_foot(2,0)) * (std::sin(PI * (i) / num11));
        //std::cout <<  (std::cos(PI * (i) / numtt)) << std::endl;
        dz_swing = -h1 / 2 * std::sin(PI * i / num11) * PI / tt1;
        ddz_swing = -h1 / 2 * std::cos(PI * i / num11) * std::pow(PI / tt1, 2);
        //std::cout << z_swing << std::endl;
      }
      else
      {
        z_swing = cur_foot(2, 0) + h2/2 * ( std::sin(0.5* PI * (i-num11) / num22));
        
        dz_swing = h2 / 2 * std::sin(PI * (i-num11) / num22) * PI / t2;
        ddz_swing = h2 / 2 * std::cos(PI * (i-num11) / num22) * std::pow(PI / t2, 2);
      }
      //if(i < num1)
     // {
     //   z_swing = pre_foot(2, 0) + h1 / 2 * (1 - std::cos(PI * i / num1));
      //  dz_swing = h1 / 2 * std::sin(PI * i / num1) * PI / t1;
      //  ddz_swing = h1 / 2 * std::cos(PI * i / num1) * std::pow(PI / t1, 2);
      //}
      //else if (i < num1 + num3)
     // {
      //  z_swing = pre_foot(2, 0) + h1;
      //  ddz_swing = 0;
     // }
     // else
     // {
     //   z_swing = nex_foot(2, 0) + h2 / 2 * (1 + std::cos(PI * (i-num1-num3) / num2));
     //   dz_swing = -h2 / 2 * std::sin(PI * (i-num1-num3) / num2) * PI / t2;
     //   ddz_swing = -h2 / 2 * std::cos(PI * (i-num1-num3) / num2) * std::pow(PI / t2, 2);
     // }

      Eigen::Matrix<double, 10, 1> swing_foot;
      swing_foot << x_swing, y_swing, z_swing, dx_swing, dy_swing, dz_swing, 
                    ddx_swing, ddy_swing, ddz_swing, d_t;
      //Eigen::Matrix<double, 10, 1> stance_foot;
      //stance_foot << cur_foot(0, 0), cur_foot(1, 0), cur_foot(2, 0), 0, 0, 0, 0, 0, 0, d_t;
    if(i < num11)
    {
      if (stance == 0)
      {
        r_foot_list.push_back(swing_foot);
        l_foot_list.push_back(stance_foot);
      }
      else
      {
        l_foot_list.push_back(swing_foot);
        r_foot_list.push_back(stance_foot);
      }
    }
    else
    {
      if (stance == 1)
      {
        r_foot_list.push_back(swing_foot);
        l_foot_list.push_back(stance_foot);
      }
      else
      {
        l_foot_list.push_back(swing_foot);
        r_foot_list.push_back(stance_foot);
      }

    }
    /*
      if (stance == 0)
      {
        r_foot_list.push_back(swing_foot);
        l_foot_list.push_back(stance_foot);
      }
      else
      {
        l_foot_list.push_back(swing_foot);
        r_foot_list.push_back(stance_foot);
      }
      */
    }
    
  }

  void PhaseSpacePlanner::LastStep()
  {
    

    double T = step_period[step-1](1, 0);
    double dt = 0.001;
    int num = T/dt;
    
    

    Eigen::Matrix<double, 1, 1> TS;
    Tstep = Tstep + T;
    TS << Tstep;
    //Tlist.push_back(TS);

    Eigen::Matrix<double, 6, 1> start = switch_list[step-1];
    Eigen::Matrix<double, 6, 1> end;
    end << apex_list[step](0, 0), apex_list[step](1, 0), apex_list[step](2, 0), 
           apex_list[step](4, 0)*cos(apex_list[step](3, 0)), 
           apex_list[step](4, 0)*sin(apex_list[step](3, 0)), 0;

    Eigen::Matrix<double, 3, 1> pre_foot = p_foot_list[step-1];
    Eigen::Matrix<double, 3, 1> cur_foot = p_foot_list[step];
    Eigen::Matrix<double, 3, 1> nex_foot;

    Eigen::Matrix<double, 2, 1> wwsq = WSQlist[step-2];

    if (stance == 0)
    {    
      nex_foot << apex_list[step](0, 0) + 0.135 * std::sin(-apex_list[step](3, 0)),
                  apex_list[step](1, 0) + 0.135 * std::cos(-apex_list[step](3, 0)), 
                  p_foot_list[step](2, 0);
    }
    else
    {
      nex_foot << apex_list[step](0, 0) - 0.135 * std::sin(-apex_list[step](3, 0)),
                  apex_list[step](1, 0) - 0.135 * std::cos(-apex_list[step](3, 0)), 
                  p_foot_list[step](2, 0);
    }
    
    Eigen::Matrix<double, 2, 1> slope = step_surface[step-1];

    double x_ddot, x_dot = start(3, 0), x = start(0, 0);
    double y_ddot, y_dot = start(4, 0), y = start(1, 0);
    double z_ddot, z_dot = start(5, 0), z = start(2, 0);

    //double x_swing, dx_swing, ddx_swing;
    //double y_swing, dy_swing, ddy_swing;
    //double z_swing, dz_swing, ddz_swing;

    Eigen::Matrix<double, 10, 1> swing_init_foot;
    if(stance == 0)
    {
        swing_init_foot=l_foot_list.back();
    }
    else
    {
        swing_init_foot=r_foot_list.back();
    }

    double x_swing0 = swing_init_foot(0,0) , dx_swing, ddx_swing;
    double y_swing0 = swing_init_foot(1,0), dy_swing, ddy_swing;
    double z_swing0 = swing_init_foot(2,0), dz_swing, ddz_swing;
    double x_swing;
    double y_swing;
    double z_swing;
    //z_swing0=0;

    double x_distance = nex_foot(0, 0) - x_swing0;// 
    double y_distance = cur_foot(1, 0) - y_swing0;
    //std::cout << x_distance << "  " << y_distance << std::endl;
    double dh = 0.3;
    double h1 = dh;
    double h2 = dh + pre_foot(2, 0) - nex_foot(2, 0);
    double t1, t2, ts;
    double vmax = 1.5;
    bool steady = false;

    if (h1 > h2)
    {
      t1 = h1 * PI / (2 * vmax);
      t2 = t1 * h2 / h1;
    }
    else
    {
      t2 = h2 * PI / (2 * vmax);
      t1 = t2 * h1 / h2;
    }
    
    if (T < t1 + t2)
    {
      h1 = (T * h1 / (h1 + h2)) * 2 * vmax / PI;
      h2 = h1 + pre_foot(2, 0) - nex_foot(2, 0);
      t1 = T * h1 / (h1 + h2);
      t2 = T * h2 / (h1 + h2);
      ts = 0;
    }
    else
    {
      steady = true;
      ts = T - t1 - t2;
    }

    int num1 = t1 / dt;
    int num2 = t2 / dt;
    int num3 = ts / dt;
    
    double w_sq = G / (apex_list[step](2, 0) - cur_foot(2, 0));
    double inc_x, inc_y;

    // step time, t1, t2
    /*
      Eigen::Matrix<double, 3, 1> time;
      time << TS, t1, t2;
      Tlist.push_back(time);
      */

    for (int i = 0; i < num; i++)
    { 
      d_t += 0.001;

      x_ddot = w_sq * (x - cur_foot(0, 0));
      inc_x = dt * x_dot + 0.5 * dt * dt * x_ddot;
      x = x + inc_x;
      x_dot = x_dot + dt * x_ddot;

      y_ddot = w_sq * (y - cur_foot(1, 0));
      inc_y = dt * y_dot + 0.5 * dt * dt * y_ddot;
      y = y + inc_y;
      y_dot = y_dot + dt * y_ddot;

      z_ddot = slope(0, 0) * x_ddot + slope(1, 0) * y_ddot;
      z = z + slope(0, 0) * inc_x + slope(1, 0) * inc_y;
      z_dot = slope(0, 0) * x_dot + slope(1, 0) * y_dot;

      Eigen::Matrix<double, 10, 1> COM;
      COM << x, y, z, x_dot, y_dot, z_dot, x_ddot, y_ddot, z_ddot, d_t;
      COM_list.push_back(COM);

      Eigen::Matrix<double, 1, 1> heading;
      heading << (direction_list[step-1](2, 0) - direction_list[step-1](1, 0))
                *(i + 1)/num + direction_list[step-1](1, 0);
      heading_list.push_back(heading);

      x_swing = x_swing0 + x_distance / 2 * (1 - std::cos(PI * i / num));
      dx_swing = x_distance / 2 * std::sin(PI * i / num) * PI / T;
      ddx_swing = x_distance / 2 * std::cos(PI * i / num) * std::pow(PI / T, 2);

      y_swing = y_swing0 + y_distance / 2 * (1 - std::cos(PI * i / num));
      dy_swing = y_distance / 2 * std::sin(PI * i / num) * PI / T;
      ddy_swing = y_distance / 2 * std::cos(PI * i / num) * std::pow(PI / T, 2);



     // if(i < num1)
     // {
       // z_swing = z_swing0 -  h1 / 2 * (1 - std::cos(PI * i / num1));
       // dz_swing = h1 / 2 * std::sin(PI * i / num1) * PI / t1;
       // ddz_swing = h1 / 2 * std::cos(PI * i / num1) * std::pow(PI / t1, 2);

    //  }
     // else if (i < num1 + num3)
     // {
      //  z_swing = pre_foot(2, 0) + h1;
      //  dz_swing = 0;
       // ddz_swing = 0;
      //}
     // else
     // {
        z_swing = nex_foot(2,0) + (z_swing0-nex_foot(2, 0)) * (std::cos(0.5*PI * (i) / num));
        dz_swing = -h2 / 2 * std::sin(PI * (i-num1-num3) / num2) * PI / t2;
        ddz_swing = -h2 / 2 * std::cos(PI * (i-num1-num3) / num2) * std::pow(PI / t2, 2);
      //}

      Eigen::Matrix<double, 10, 1> swing_foot;
      swing_foot << x_swing, y_swing, z_swing, dx_swing, dy_swing, dz_swing, 
                    ddx_swing, ddy_swing, ddz_swing, d_t;
      Eigen::Matrix<double, 10, 1> stance_foot;
      stance_foot << cur_foot(0, 0), cur_foot(1, 0), cur_foot(2, 0), 0, 0, 0, 0, 0, 0, d_t;

      if (stance == 1)
      {
        r_foot_list.push_back(swing_foot);
        l_foot_list.push_back(stance_foot);
      }
      else
      {
        l_foot_list.push_back(swing_foot);
        r_foot_list.push_back(stance_foot);
      }
    }
  }

  void PhaseSpacePlanner::End()
  {
    stance = 2;
  }

  void PhaseSpacePlanner::InitObstacle(Eigen::Matrix<double, 3, 1> obs_init,Eigen::Matrix<double, 3, 1> obs2_init)
  {
    Eigen::Matrix<double, 6, 1> obstacle;
    obstacle << obs_init(0, 0), obs_init(1, 0), obs_init(2, 0), obs2_init(0, 0), obs2_init(1, 0), obs2_init(2, 0);
    obstacle_list.push_back(obstacle);
  }

  void PhaseSpacePlanner::UpdateObstacle(Eigen::Matrix<double, 3, 1> obs, Eigen::Matrix<double, 3, 1> obs2)
  {
    double T = step_period[step-1](0, 0) + step_period[step-1](1, 0);
    double dt = 0.001;
    int num = T/dt;

    Eigen::Matrix<double, 6, 1> pre = obstacle_list.back();

    for (int i = 0; i < num; i++)
    {
      Eigen::Matrix<double, 6, 1> obstacle;
      obstacle << (obs(0, 0) - pre(0, 0))*(i+1)/num + pre(0, 0),
                  (obs(1, 0) - pre(1, 0))*(i+1)/num + pre(1, 0),
                  (obs(2, 0) - pre(2, 0))*(i+1)/num + pre(2, 0),
                  (obs2(0, 0) - pre(3, 0))*(i+1)/num + pre(3, 0),
                  (obs2(1, 0) - pre(4, 0))*(i+1)/num + pre(4, 0),
                  (obs2(2, 0) - pre(5, 0))*(i+1)/num + pre(5, 0);
      obstacle_list.push_back(obstacle);
    }
  }

  void PhaseSpacePlanner::Stand(Eigen::Matrix<double, 3, 1> obs,  Eigen::Matrix<double, 3, 1> obs2)
  {
    int num = 1000;

    Eigen::Matrix<double, 6, 1> pre = obstacle_list.back();

    for (int i = 0; i < num; i++)
    {
      Eigen::Matrix<double, 6, 1> obstacle;
      obstacle << (obs(0, 0) - pre(0, 0))*(i+1)/num + pre(0, 0),
                  (obs(1, 0) - pre(1, 0))*(i+1)/num + pre(1, 0),
                  (obs(2, 0) - pre(2, 0))*(i+1)/num + pre(2, 0),
                  (obs2(0, 0) - pre(3, 0))*(i+1)/num + pre(3, 0),
                  (obs2(1, 0) - pre(4, 0))*(i+1)/num + pre(4, 0),
                  (obs2(2, 0) - pre(5, 0))*(i+1)/num + pre(5, 0);
      obstacle_list.push_back(obstacle);
      
      Eigen::Matrix<double, 10, 1> COM = COM_list.back();
      COM_list.push_back(COM);

      Eigen::Matrix<double, 10, 1> l_foot = l_foot_list.back();
      l_foot_list.push_back(l_foot);

      Eigen::Matrix<double, 10, 1> r_foot = r_foot_list.back();
      r_foot_list.push_back(r_foot);

      Eigen::Matrix<double, 1, 1> heading = heading_list.back();
      heading_list.push_back(heading);
      
      
    }
  }

void PhaseSpacePlanner::InitControlMap()
  {
  
  //std::map<std::pair<double,double>, double> map_of_control;
  const std::string full_name = drake::FindResourceOrThrow("drake/safe-nav-loco/fhws2_2.txt");
  std::ifstream infile(full_name); // Need to run this code under the same folder.
  // infile.open("test.txt");
  std::vector<double> traj = {1, 1, 1};
  std::string line;
  int row = 0;
 

  // std::vector<std::vector<double>> buff(num_state, std::vector<double>(traj_length, 0));
  while (std::getline(infile, line)) {
    
    std::istringstream ss(line);
    std::string number;
    int col = 0;
    while(std::getline(ss, number, ',')) {
      // std::cout << std::stod(number) << '\n';
      traj[col] = std::stod(number);
      //std::cout << traj[col] << std::endl;
      col++;
    }
      
      //std::cout << traj[0] << " " << traj[1] << " " << traj[2] << std::endl;
      std::pair<int, int> key_of_control(traj[0]*1000, traj[1]*1000);
      map_of_control_fhws[key_of_control] = traj[2];
      row++;
  }
  infile.close();
  std::cout << "Finish read fhws.\n";
    

  const std::string full_name1 = drake::FindResourceOrThrow("drake/safe-nav-loco/shws2_2.txt");
  std::ifstream infile2(full_name1); // Need to run this code under the same folder.
  // infile.open("test.txt");
  std::vector<double> traj2 = {1, 1, 1};
  std::string line2;
  int roww = 0;
 

  // std::vector<std::vector<double>> buff(num_state, std::vector<double>(traj_length, 0));
  while (std::getline(infile2, line2)) {
    
    std::istringstream ss(line2);
    std::string number2;
    int coll = 0;
    while(std::getline(ss, number2, ',')) {
      // std::cout << std::stod(number) << '\n';
      traj2[coll] = std::stod(number2);
      //std::cout << traj[col] << std::endl;
      coll++;
      
    }
      
      //std::cout << traj[0] << " " << traj[1] << " " << traj[2] << std::endl;
      std::pair<int, int> key_of_control(traj2[0]*1000, traj2[1]*1000);
      map_of_control_shws[key_of_control] = traj2[2];
      roww++;
  }
  infile2.close();
  std::cout << "Finish read shws.\n";

  /*
   int xx_c = 202;
   int dxx_c = 788;
   std::pair<int, int> key_of_control(xx_c, dxx_c);
   double u = map_of_control_shws[key_of_control];
   std::cout << u << std::endl;
  */

  }


void PhaseSpacePlanner::perturbed_traj() 
  {
    double T = step_period[step-2](1, 0) + step_period[step-1](0, 0);
    double dt = 0.001; 
    int num = T/dt;
    double tt1=step_period[step-1](0, 0);
    double tt2=step_period[step-1](1, 0);
    Eigen::Matrix<double, 1, 1> TS;
    Tstep = Tstep + T;
    TS << Tstep;
    //Tlist.push_back(TS);
    //std::cout << "num= " << num << std::endl;

 
    Eigen::Matrix<double, 10, 1> cont_start = COM_list.back();
    Eigen::Matrix<double, 6, 1> start = apextraj_list[step-1];//apextraj_list[step-1];//switch_list[step-2];
    Eigen::Matrix<double, 6, 1> end = switch_list[step-1];//apextraj_list[step];//switch_list[step-1];

    Eigen::Matrix<double, 3, 1> pre_foot = p_foot_list[step-2];
    Eigen::Matrix<double, 3, 1> cur_foot = p_foot_list[step-1];
    Eigen::Matrix<double, 3, 1> nex_foot = p_foot_list[step];

    Eigen::Matrix<double, 2, 1> slope1 = step_surface[step-2];
    Eigen::Matrix<double, 2, 1> slope2 = step_surface[step-1];

    Eigen::Matrix<double, 2, 1> wwsq = WSQlist[step-2];

    double x_ddot, x_dot = cont_start(3, 0), x = cont_start(0, 0);
    double y_ddot, y_dot = cont_start(4, 0), y = cont_start(1, 0);
    double z_ddot, z_dot = cont_start(5, 0), z = cont_start(2, 0);

    //double init_x_dot = x_dot;
    double init_x= x;


    Eigen::Matrix<double, 10, 1> swing_init_foot;
    if(stance == 1)
    {
        swing_init_foot=l_foot_list.back();
    }
    else
    {
        swing_init_foot=r_foot_list.back();
    }

    double x_swing0 = swing_init_foot(0,0) , dx_swing, ddx_swing;
    double y_swing0 = swing_init_foot(1,0), dy_swing, ddy_swing;
    double z_swing0 = swing_init_foot(2,0), dz_swing, ddz_swing;
    double x_swing;
    double y_swing;
    double z_swing;

    //x_swing0 = 0;
    //y_swing0 = 0;

    double x_distance = nex_foot(0, 0) - pre_foot(0, 0); 
    double y_distance = nex_foot(1, 0) - pre_foot(1, 0);
    double dh = 0.3;
    double h1 = dh;
    double h2 = dh + pre_foot(2, 0) - nex_foot(2, 0);
    double t1, t2, ts;
    double vmax = 1.5;
    bool steady = false;

    //double x_c ;
    //double dx_c ;
    //double rdx_c ;//= 0.004 * std::round(dx_c / 0.004);
    //std::cout << rdx_c << std::endl;
    //std::pair<double, double> key_of_control(x_c, rdx_c);
    //double u = map_of_control_fhws[key_of_control];
    //std::cout << u << std::endl;

    if (h1 > h2)
    {
      t1 = h1 * PI / (2 * vmax);
      t2 = t1 * h2 / h1;
    }
    else
    {
      t2 = h2 * PI / (2 * vmax);
      t1 = t2 * h1 / h2;
    }
    
    if (T < t1 + t2)//(T < t1 + t2)
    {
      h1 = (T * h1 / (h1 + h2)) * 2 * vmax / PI;
      h2 = h1 + pre_foot(2, 0) - nex_foot(2, 0);
      t1 = T * h1 / (h1 + h2);
      t2 = T * h2 / (h1 + h2);
      ts = 0;
    }
    else
    {
      steady = true;
      ts = T-t1-t2;//T - t1 - t2;
    }

    int num0 = step_period[step-2](1, 0) / dt;
    //int num1 = t1 / dt;
    //int num2 = t2 / dt;
    //int num3 = ts / dt;

    double tt3=step_period[step-2](1, 0);
    int num4 = tt3/dt;
    int numtt= (tt1+tt2)/dt;
    int num11= (tt1)/dt;
    int num22= tt2/dt;

    
    Eigen::Matrix<double, 10, 1> stance_foot;

      double backward_flag = 1;
   // double w_sq = wwsq(1, 0);
    double w_sq11 = 1000;
    double w_sq22 = 1000;
    double inc_x, inc_y;
    //int i =0 ;

    double fw_num = 0 ;
    double bkwrd_num = 0;
    for (int i = 0; i < numtt; i++)  //(x <= nex_foot(0,0) )//(int i = 0; i < num; i++)
    { 


      int x_c = (0.002 * std::round((x-init_x) / 0.002))*1000;
      int dx_c = (0.004 * std::round((x_dot) / 0.004))*1000;
     
      std::pair<int, int> key_of_control(x_c, dx_c);
      //std::pair<int,int> key_of_controll(92, 1096);
      w_sq11 = map_of_control_fhws[key_of_control];
      w_sq22 = map_of_control_shws[key_of_control];

        
      double w_sq1 = w_sq11*w_sq11;
      double w_sq2 = w_sq22*w_sq22;

      //std::cout << "x_c= " << key_of_control.first << " dx_c= " << key_of_control.second << " w_sq1= " << w_sq11 << " w_sq2= " << w_sq22 << std::endl;



      
      if (i < num11)
      { 
       
      Eigen::Matrix<double, 2, 1> noise1 = noise1_list[fw_num];

      Eigen::Matrix<double, 1, 1> w1_sq = w1_frwrd[fw_num];
      w_sq1 = w1_sq(0,0);

       x_ddot = w_sq1 * (x - cur_foot(0, 0));
       inc_x = dt * x_dot + 0.5 * dt * dt * x_ddot;
       x = x + inc_x + noise1(0, 0);
       x_dot = (x_dot + dt * x_ddot) + noise1(1, 0);

       y_ddot = w_sq1 * (y - cur_foot(1, 0));
       inc_y = dt * y_dot + 0.5 * dt * dt * y_ddot;
       y = y + inc_y;
       y_dot = y_dot + dt * y_ddot;



        z_ddot = slope1(0, 0) * x_ddot + slope1(1, 0) * y_ddot;
        z = z + slope1(0, 0) * inc_x + slope1(1, 0) * inc_y;
        z_dot = slope1(0, 0) * x_dot + slope1(1, 0) * y_dot;

        Eigen::Matrix<double, 1, 1> heading;
        heading << (direction_list[step-2](2, 0) - direction_list[step-2](1, 0))
                  *(i + 1)/num0 
                  + direction_list[step-2](1, 0);
        heading_list.push_back(heading);

        backward_flag=1;
        fw_num = fw_num + 1; 

       // x_swing = pre_foot(0, 0) + x_distance / 2 * (1 - std::cos(PI * (i+num4) / numtt));
        x_swing = nex_foot(0, 0) + (x_swing0 - nex_foot(0, 0)) * (std::cos(0.5*PI * (i+num4) / numtt));
        dx_swing = x_distance / 2 * std::sin(PI * i / numtt) * PI / T;
        ddx_swing = x_distance / 2 * std::cos(PI * i / numtt) * std::pow(PI / T, 2);

        //y_swing = pre_foot(1, 0) + y_distance / 2 * (1 - std::cos(PI * (i+num4) / numtt));
        y_swing = nex_foot(1, 0) + (y_swing0 - nex_foot(1, 0)) * (std::cos(0.5*PI * (i+num4) / numtt));
        dy_swing = y_distance / 2 * std::sin(PI * i / numtt) * PI / T;
        ddy_swing = y_distance / 2 * std::cos(PI * i / numtt) * std::pow(PI / T, 2);

        //Eigen::Matrix<double, 10, 1> stance_foot;
        stance_foot << cur_foot(0, 0), cur_foot(1, 0), cur_foot(2, 0), 0, 0, 0, 0, 0, 0, d_t;
      }
      else
      { 
          Eigen::Matrix<double, 2, 1> noise2 = noise2_list[bkwrd_num];

          Eigen::Matrix<double, 1, 1> w_sq = w2_bkwrd[bkwrd_num];
          
          w_sq2 = w_sq(0,0);
          //std::cout<< w_sq2 << std::endl;

          x_ddot = w_sq2 * (x - nex_foot(0, 0));
          inc_x = dt * x_dot + 0.5 * dt * dt * x_ddot;
          x = x + inc_x + noise2(0, 0);
          x_dot = (x_dot + dt * x_ddot) + noise2(1, 0);

          y_ddot = w_sq2 * (y - nex_foot(1, 0));
          inc_y = dt * y_dot + 0.5 * dt * dt * y_ddot;
          y = y + inc_y;
          y_dot = y_dot + dt * y_ddot;


        z_ddot = slope2(0, 0) * x_ddot + slope2(1, 0) * y_ddot;
        z = z + slope2(0, 0) * inc_x + slope2(1, 0) * inc_y;
        z_dot = slope2(0, 0) * x_dot + slope2(1, 0) * y_dot;
        Eigen::Matrix<double, 1, 1> heading;
        heading << (direction_list[step-1](1, 0) - direction_list[step-1](0, 0))
                  *(i  + 1 - num0)/(num - num0)
                  + direction_list[step-1](0, 0);
        heading_list.push_back(heading);
        bkwrd_num = bkwrd_num +1;

        x_swing = cur_foot(0, 0) + x_distance / 4 * (1 - std::cos(PI * (i-num11) / num22));
        dx_swing = x_distance / 4 * std::sin(PI * (i-num11)/ num22) * PI / T;
        ddx_swing = x_distance / 4 * std::cos(PI * (i-num11) / num22) * std::pow(PI / T, 2);

        y_swing = cur_foot(1, 0) + y_distance / 4 * (1 - std::cos(PI * (i-num11)/ num22));
        dy_swing = y_distance / 4 * std::sin(PI * (i-num11) / num22) * PI / T;
        ddy_swing = y_distance / 4 * std::cos(PI * (i-num11) / num22) * std::pow(PI / T, 2);

      //Eigen::Matrix<double, 10, 1> stance_foot;
        stance_foot << nex_foot(0, 0), nex_foot(1, 0), nex_foot(2, 0), 0, 0, 0, 0, 0, 0, d_t;
      }

      Eigen::Matrix<double, 10, 1> COM;
      COM << x, y, z, x_dot, y_dot, z_dot, x_ddot, y_ddot, z_ddot, d_t;
      COM_list.push_back(COM);

      


      if(i < num11)
      {
        z_swing =nex_foot(2,0)  + (z_swing0 - nex_foot(2,0)) * (std::cos(0.5 * PI * (i) / num11));
        //std::cout <<  (std::cos(PI * (i) / numtt)) << std::endl;
        dz_swing = -h1 / 2 * std::sin(PI * i / num11) * PI / tt1;
        ddz_swing = -h1 / 2 * std::cos(PI * i / num11) * std::pow(PI / tt1, 2);
        /*
        z_swing =  z_swing0  * (std::cos(PI * (i) / numtt));
        //std::cout <<  z_swing0 << std::endl;
        dz_swing = -h1 / 2 * std::sin(PI * i / numtt) * PI / tt1;
        ddz_swing = -h1 / 2 * std::cos(PI * i / numtt) * std::pow(PI / tt1, 2);
        */
      }
     // else if (i < num1 + num3)
      //{
       // z_swing = pre_foot(2, 0) + h1;
      // // dz_swing = 0;
      //  ddz_swing = 0;
     // }
      else
      {
        z_swing = cur_foot(2, 0) + h2/2 * (1 - std::cos(PI * (i-num11) / num22));
        //std::cout << (1 - std::cos(PI * (i-num11) / num22)) << std::endl;
        dz_swing = h2 / 2 * std::sin(PI * (i-num11) / num22) * PI / t2;
        ddz_swing = h2 / 2 * std::cos(PI * (i-num11) / num22) * std::pow(PI / t2, 2);
      }

      Eigen::Matrix<double, 10, 1> swing_foot;
      swing_foot << x_swing, y_swing, z_swing, dx_swing, dy_swing, dz_swing, 
                    ddx_swing, ddy_swing, ddz_swing, d_t;
    if(i < num11)
    {
      if (stance == 0)
      {
        r_foot_list.push_back(swing_foot);
        l_foot_list.push_back(stance_foot);
      }
      else
      {
        l_foot_list.push_back(swing_foot);
        r_foot_list.push_back(stance_foot);
      }
    }
    else
    {
      if (stance == 1)
      {
        r_foot_list.push_back(swing_foot);
        l_foot_list.push_back(stance_foot);
      }
      else
      {
        l_foot_list.push_back(swing_foot);
        r_foot_list.push_back(stance_foot);
      }

    }
      d_t += 0.001;
    }
    //std::cout << fw_num << " ::::: " << bkwrd_num << std::endl;
  }

void PhaseSpacePlanner::UpdateKeyframe_pert() 
  {
    double COS = std::cos(X_apex(3, 0) + prim(1, 0));
    double SIN = std::sin(X_apex(3, 0) + prim(1, 0));

    double s1, s1_dot, s1_ddot, l1, l1_dot,l1_ddot, v1,v1_dot, v1_ddot, s1_foot, l1_foot, v1_foot;
    //double sign;
    double s2;
    double s2_dot, s2_ddot, l2, l2_dot, l2_ddot, v2, v2_dot, v2_ddot;
    double phi = 0 ;//- waypoint(3,0);
    // Foot2
    double s2_foot, l2_foot, v2_foot;

    double x;
    double xd;
       
    double fine_num=0;
    // Switch
    double s_switch, ds_switch, l_switch, dl_switch, v_switch, dv_switch;

    // Forward-Backward Prop for Sagittal Switch
    double eps, forward_num, backward_num, aq, bq, w1_sq, w2_sq;
    
    double w_sq11 =1000; 
    double w_sq22 =1000;
      
    double new_foot;
   

    Eigen::Matrix<double, 10, 1> cont_start = COM_list.back();

    prim(3, 0) = cont_start(3, 0)*std::cos(prim(1, 0)-phi); //NEXT APEX VELOCITY NOMINAL 

      
          s1 = (cont_start(0, 0) - X_d(0, 0))*COS + 
                      (cont_start(1, 0) - X_d(1, 0))*SIN;
          s1_dot = cont_start(3, 0)*std::cos(prim(1, 0)-phi);
          
          x=s1;
          xd=s1_dot;

          l1 = -(cont_start(0, 0) - X_d(0, 0))*SIN + 
                       (cont_start(1, 0) - X_d(1, 0))*COS;
          l1_dot = -cont_start(3, 0)*std::sin(prim(1, 0)-phi);
          
          
        

          v1 = cont_start(2, 0);

          // Foot1 adjustment to new local coordinates
           
          if (startflag == 1)
          { 
            if (stance == 1)
            {
              s1_foot = (p_foot(0, 0) - X_d(0, 0))*COS + (p_foot(1, 0) - X_d(1, 0))*SIN;
              l1_foot = (p_foot(0, 0) - X_d(0, 0))*SIN + (p_foot(1, 0) - X_d(1, 0))*COS;
              v1_foot = p_foot(2, 0); 


            }
            else
            {
              s1_foot = (p_foot(0, 0) - X_d(0, 0))*COS + (p_foot(1, 0) - X_d(1, 0))*SIN;
              l1_foot = -(p_foot(0, 0) - X_d(0, 0))*SIN + (p_foot(1, 0) - X_d(1, 0))*COS;
              v1_foot = p_foot(2, 0); 


            }
            
            startflag = 0 ;
          }
          else
          {
            s1_foot = (p_foot(0, 0) - X_d(0, 0))*COS + (p_foot(1, 0) - X_d(1, 0))*SIN;
            l1_foot = -(p_foot(0, 0) - X_d(0, 0))*SIN + (p_foot(1, 0) - X_d(1, 0))*COS;
            v1_foot = p_foot(2, 0); 
          }
         
          

            s2 = s1 + prim(0, 0) - ((cont_start(0, 0) - X_d(0, 0))*std::cos(X_apex(3, 0) + prim(1, 0)+phi) + (cont_start(1, 0) - X_d(1, 0))*std::sin(X_apex(3, 0) + prim(1, 0)+phi))  ; 

          

          s2_dot = prim(3, 0);
                    
          l2_dot = 0;
          
          
          v2 = v1_foot + prim(2, 0) +  prim(4, 0); 
          
          //std::cout << "delta y2=" << l1_foot - l1 << std::endl;
          //std::cout << "delta y1=" << l1 << std::endl;
          // Foot2
          double og_s2_foot = s2;
          s2_foot = s2;
          // std::cout << "s2= " << s2 << " s2_foot= "<< s2_foot << std::endl;
          double pos_guard = 0.5*(((s2_dot*s2_dot - s1_dot*s1_dot)/(3.1*3.1))/(s2_foot-s1_foot) + (s1_foot + s2_foot));//(s2-s1)*0.5 ;
          //std::cout << "s2= " << s2 << " s1 = "<< s1 << " pos_guard = " << pos_guard << std::endl;


          if (stance == 1)
          {
            l2_foot = -0.135;

          }
          else
          {
            l2_foot = 0.135 ;

          }
               
          // Forward-Backward Prop for Sagittal Switch
          eps = 0.001;
          forward_num = 0;
          backward_num = 0;
          aq = (v2 - v1) / (s2 - s1); 
          bq = 0;

          double controllable_flag = 1;
          double inc_s1, inc_l1, inc_s2;
          
          double app_pert =   -((0.1 + 0.1) * ( static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX) ) -0.1)*2;
          double app_pert_dot= -((0.2 + 0.2) * ( static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX) ) -0.2)*2.5;

          while (s2 <= s2_foot ) 
          {
            // Forward Prop

            // ROCS CONTROL

            double x_pert = 0;//(0.005 + 0.005) * ( static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX) ) -0.005;
            double xd_pert =0;//(0.005 + 0.005) * ( static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX) ) -0.005;
            if (forward_num == 200)
            {
            x_pert  = app_pert;
            xd_pert = app_pert_dot;
           
            }

            if(controllable_flag)
            {

              int x_c = (0.002 * std::round((x+x_pert) / 0.002))*1000;
              int dx_c = (0.004 * std::round((xd+xd_pert) / 0.004))*1000;
              std::pair<int, int> key_of_control(x_c, dx_c);
              w_sq11 = map_of_control_fhws[key_of_control];
              w_sq22 = map_of_control_shws[key_of_control];
              w1_sq = w_sq11*w_sq11;
              w2_sq = w_sq22*w_sq22;

            }

            


            if(w_sq11 + w_sq22 < 1 )
            {

              controllable_flag = 0;
              //std::cout << "capture" << std::endl;
              //double x_pert = 0;//(0.005 + 0.005) * ( static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX) ) -0.005;
              //double xd_pert =0;
              //Eigen::Matrix<double, 2, 1> noise1;
              //noise1 << 0, 0;//x_pert, xd_pert;
              //noise1_list.insert(noise1_list.begin() + forward_num, noise1);
                
            if(s1 < pos_guard)
            {

              //std::cout << s1_dot << std::endl;
              
              Eigen::Matrix<double, 2, 1> noise1;
              noise1 << x_pert, xd_pert;
              noise1_list.insert(noise1_list.begin() + forward_num, noise1);
              Eigen::Matrix<double, 1, 1> w1;
              w1 << 3.1*3.1;
              //std::cout << "here1" << std::endl;
              //w1_frwrd.push_back(w1);
              w1_frwrd.insert(w1_frwrd.begin() + forward_num, w1);
                          //  Eigen::Matrix<double, 1, 1> w1;
              //w1 << w1_sq;
              //w1_frwrd.push_back(w1);
              //w1_frwrd.insert(w1_frwrd.begin() + forward_num, w1);
              //std::cout << "here2" << std::endl;
              //forward propagation
              w1_sq = 3.1*3.1;
              forward_num = forward_num + 1;
              s1_ddot = w1_sq * (s1 - s1_foot);
              inc_s1 = eps * s1_dot + 0.5 * eps * eps * s1_ddot;
              s1 = s1 + inc_s1 + x_pert;
              s1_dot = (s1_dot + eps * s1_ddot) + xd_pert;

              l1_ddot = w1_sq * (l1 - l1_foot);
              inc_l1 = eps * l1_dot + 0.5 * eps * eps * l1_ddot;
              l1 = l1 + inc_l1;
              l1_dot = l1_dot + eps * l1_ddot;

              v1_ddot = aq * s1_ddot + bq * l1_ddot;
              v1 = v1 + aq * inc_s1 + bq * inc_l1;
              v1_dot = aq * s1_dot + bq * l1_dot;

              x=s1;
              xd=s1_dot;

              s2= x;
              s2_dot = xd;//std::cout << "forward_num "<< std::endl;
              //td::cout << "fhws" << std::endl;
              //std::cout << "here3" << std::endl;
            } 
            else
            {

              ///std::cout << "capt SHWS" << std::endl;
              //std::cout << "shws" << std::endl;
              new_foot = pos_guard + 1/(3.1)*std::sqrt((s1_dot*s1_dot - prim(3, 0)*prim(3, 0)));
              s2_foot = new_foot;//pos_guard + 1/(3.1)*std::sqrt((s1_dot*s1_dot - prim(3, 0)*prim(3, 0)));
              //std::cout << "new foot1 = " << s1_dot*s1_dot - prim(3, 0)*prim(3, 0) << std::endl ;
              //std::cout << "x = s2 = " << s2 << std::endl;
              //double x_pert = 0;//(0.005 + 0.005) * ( static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX) ) -0.005;
              //double xd_pert = 0;//(0.002 + 0.002) * ( static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX) ) -0.002;
              Eigen::Matrix<double, 2, 1> noise2;
              noise2 << x_pert, xd_pert;
              noise2_list.insert(noise2_list.begin() + backward_num, noise2);


              Eigen::Matrix<double, 1, 1> w2;
              w2 << 3.1*3.1;
              w2_sq = 3.1*3.1;
              //w2_bkwrd.push_back(w2);
              w2_bkwrd.insert(w2_bkwrd.begin() + backward_num, w2);
              backward_num = backward_num + 1;
              s2_ddot = w2_sq * (s2 - s2_foot);
              inc_s2 =  eps * s2_dot + 0.5 * eps * eps * s2_ddot;
              s2 = s2 + inc_s2 + x_pert;
              s2_dot = (s2_dot + eps * s2_ddot) + xd_pert;

              v2_ddot = aq * s2_ddot;
              v2 = v2 + aq * inc_s2;
              v2_dot = aq * s2_dot;

              x=s2;
              xd=s2_dot;

            }

            }
             
            else if(!w2_sq && controllable_flag) 
            {


              std::cout << "cont" << std::endl;
              //x_pert = 0;//(0.005 + 0.005) * ( static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX) ) -0.005;
              //xd_pert =0;//(0.005 + 0.005) * ( static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX) ) -0.005;
              //if (forward_num == 160)
              //{
              //x_pert = ((0.01 + 0.01) * ( static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX) ) -0.01)*5;
              //xd_pert =((0.2 + 0.2) * ( static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX) ) -0.2)*5;
              // }
              Eigen::Matrix<double, 2, 1> noise1;
              noise1 << x_pert, xd_pert;
              noise1_list.insert(noise1_list.begin() + forward_num, noise1);
              

              Eigen::Matrix<double, 1, 1> w1;
              w1 << w1_sq;
              //w1_frwrd.push_back(w1);
              w1_frwrd.insert(w1_frwrd.begin() + forward_num, w1);

              forward_num = forward_num + 1;
              s1_ddot = w1_sq * (s1 - s1_foot);
              inc_s1 = eps * s1_dot + 0.5 * eps * eps * s1_ddot;
              s1 = s1 + inc_s1 + x_pert;
              s1_dot = (s1_dot + eps * s1_ddot) + xd_pert;

              l1_ddot = w1_sq * (l1 - l1_foot);
              inc_l1 = eps * l1_dot + 0.5 * eps * eps * l1_ddot;
              l1 = l1 + inc_l1;
              l1_dot = l1_dot + eps * l1_ddot;

              v1_ddot = aq * s1_ddot + bq * l1_ddot;
              v1 = v1 + aq * inc_s1 + bq * inc_l1;
              v1_dot = aq * s1_dot + bq * l1_dot;

              x=s1;
              xd=s1_dot;

               s2= x;
               s2_dot = xd;//std::cout << "forward_num "<< std::endl;
            } 
            // Backward Prop
            else if (!w1_sq && controllable_flag) 
            {
              std::cout << "cont2" << std::endl;
              //double x_pert = 0;//(0.005 + 0.005) * ( static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX) ) -0.005;
              //double xd_pert = 0;//(0.002 + 0.002) * ( static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX) ) -0.002;
              Eigen::Matrix<double, 2, 1> noise2;
              noise2 << x_pert, xd_pert;
              noise2_list.insert(noise2_list.begin() + backward_num, noise2);


              Eigen::Matrix<double, 1, 1> w2;
              w2 << w2_sq;
              //w2_bkwrd.push_back(w2);
              w2_bkwrd.insert(w2_bkwrd.begin() + backward_num, w2);
              //w2_bkwrd[backward_num] = w2_sq;
              backward_num = backward_num + 1;
              s2_ddot = w2_sq * (s2 - s2_foot);
              inc_s2 =  eps * s2_dot + 0.5 * eps * eps * s2_ddot;
              s2 = s2 + inc_s2 + x_pert;
              s2_dot = (s2_dot + eps * s2_ddot) + xd_pert;

              v2_ddot = aq * s2_ddot;
              v2 = v2 + aq * inc_s2;
              v2_dot = aq * s2_dot;

              x=s2;
              xd=s2_dot;

              
              //std::cout << "w2 backward= " << w2_sq << std::endl;
            }
            
          }

          s2_dot = prim(3,0);
          std::cout << "backward_num " << backward_num << std::endl;
          std::cout << "forward_num " << forward_num << std::endl;
          std::cout << "new foot " << s2_foot << std::endl;
          s_switch = (s1 + s2) / 2;
          ds_switch = (s1_dot + s2_dot) / 2;

          l_switch = l1;
          dl_switch = l1_dot;

          v_switch = (v1 + v2) / 2;
          dv_switch = (v1_dot + v2_dot) / 2;


          // Newton-Raphson Search for Lateral Foot Placement
          int n = 1, n_max = 150;
          double max_tol = 0.001;
          double pre_foot, pre_dot;
          Eigen::Matrix<double, 3, 1> res;

          res = ForwardProp(l2_foot, l1, l1_dot, (prim(4, 0)), aq, eps, backward_num); 
          l2_dot = res(1, 0);
          l2_ddot = 0.002;
          //std::cout << "l2_dot= " << l2_dot << std::endl;
          double inc_p;
          while (n < n_max && std::abs(l2_dot) > max_tol)
          {
            
            pre_foot = l2_foot;
            l2_foot = l2_foot - l2_dot/l2_ddot;
            pre_dot = l2_dot;
            // Starting Pos
 
            for (int i = 0; i < backward_num; i++) //Newton-Raphson Search
              {   
                if (i == 0)
                {
                      l2_dot = l1_dot;
                      l2= l1;    
                }

                Eigen::Matrix<double, 1, 1> w_sq = w2_bkwrd[i];
                l2_ddot = w_sq(0,0) * (l2 - l2_foot);
                //l2_ddot = w2_bkwrd[i] * (l2 - l2_foot);
                inc_p = eps * l2_dot + 0.5 * eps * eps * l2_ddot;
                l2 = l2 + inc_p;
                l2_dot = l2_dot + eps * l2_ddot;
              }

            //l2_dot = res(1, 0);
            l2_ddot = (l2_dot - pre_dot) / (l2_foot - pre_foot);
            n = n + 1;
            //std::cout << "here" << std::endl;
          }
         

          s2 = s2_foot;
          s2_dot = prim(3, 0);

          //l2 = res(0, 0);
          //l2_dot = res(1, 0);

          v2 = p_foot(2, 0) + prim(2, 0) + prim(4, 0); 
          v2_dot = aq * s2_dot; 

        //  std::cout << "l2= " << l2 << std::endl;
         // std::cout << "l2_dot= " << l2_dot << std::endl;
         // std::cout << "dy2_n= " << l2_foot << std::endl;
          //std::cout << "used vn= "<< prim(3,0) << std::endl;


        

        //std::cout <<  "step length= "<< prim(0,0) << std::endl;
       

     Eigen::Matrix<double, 8, 1> psp;
     psp << (X_apex(0, 0) - X_d(0, 0))*COS + (X_apex(1, 0) - X_d(1, 0))*SIN, s1_foot, X_apex(4, 0)*std::cos(prim(1, 0)-phi), s2, s2_foot, s2_dot, eps*forward_num, eps*backward_num;
     psp_list.push_back(psp);
    //goodlateral = 1;
    // p_foot update (next step)
    p_foot(0, 0) = s2_foot*COS - l2_foot*SIN + X_d(0, 0);
    p_foot(1, 0) = s2_foot*SIN + l2_foot*COS + X_d(1, 0);
    p_foot(2, 0) = p_foot(2, 0) + prim(2, 0); 
    
    v2_foot = p_foot(2, 0);
    // X_switch
    X_switch(0, 0) = s_switch*COS - l_switch*SIN + X_d(0, 0);
    X_switch(1, 0) = s_switch*SIN + l_switch*COS + X_d(1, 0);
    X_switch(2, 0) = v_switch;
    X_switch(3, 0) = ds_switch*COS - dl_switch*SIN;
    X_switch(4, 0) = ds_switch*SIN + dl_switch*COS;
    X_switch(5, 0) = dv_switch;
    
    // apex trajectory keyframe1
    apextraj(0, 0)= X_apex(0, 0); //xapex
    apextraj(1, 0)= X_apex(1, 0);//yapex
    apextraj(2, 0)= X_apex(2, 0);  //zapex
    apextraj(3, 0)= X_apex(4, 0)*std::cos(X_apex(3, 0)+phi); //xd
    apextraj(4, 0)= X_apex(4, 0)*std::sin(X_apex(3, 0)+phi); //yd
    apextraj(5, 0)= aq*apextraj(3, 0); //zd

        
    // Keyframe 
    X_apex(0, 0) = s2*COS - l2*SIN + X_d(0, 0);
    X_apex(1, 0) = s2*SIN + l2*COS + X_d(1, 0);
    X_apex(2, 0) = p_foot(2, 0) + prim(4, 0); 
    X_apex(3, 0) += prim(1, 0) ; //X_d(3, 0) + prim(1, 0) + phi;
    X_apex(4, 0) = prim(3, 0);
    double delta_sag = 0;

   
      if((new_foot - og_s2_foot) < -0.05195)
      {
        delta_sag = -1;
      }
      if((new_foot - og_s2_foot) > 0.05195)
      {
        delta_sag =-1;
        std::cout << "here" <<std::endl;
      }
    


      if (prim(0, 0) > 0.4 && prim(0, 0) < 0.46)
      {
        sag = 4;
      }
      else if (prim(0, 0) > 0.46)
      {
        sag = 5;
      }
      else if (prim(0, 0) > 0.3 && prim(0, 0) < 0.4)
      {
        sag = 3;
      }
      else if (prim(0, 0) > 0.2 && prim(0, 0) < 0.3)
      {
        sag = 3;
      }

      sag = sag+delta_sag;
      std::cout << sag << "  " << delta_sag << "  " << new_foot - og_s2_foot  << std::endl;


    if ((prim(1, 0) != 0) && ( (std::cos(X_apex(3,0)) > 0.95) || (std::cos(X_apex(3,0)) < -0.95) || (std::sin(X_apex(3,0))>0.95) || (std::sin(X_apex(3,0)) < -0.95)  )) //((prim(1, 0) == 0) && (X_apex(3,0) != prev_heading))
    {

        //std::cout << "**here**" << std::endl << std::endl;


      if (stance == 0) //fine cell adjusment 
      {
          if (l2 > 0.314)
        {
           X_d(0, 0) += s2_foot*std::cos(X_apex(3, 0)) -0.208*std::sin(X_apex(3, 0)); //X_apex(0, 0);
           X_d(1, 0) += s2_foot*std::sin(X_apex(3, 0)) +0.208*std::cos(X_apex(3, 0)); //X_apex(1, 0);
           X_d(2, 0) =  p_foot(2, 0) + prim(4, 0); 
           X_d(3, 0) += prim(1, 0);
           X_d(4, 0) = prim(3, 0);
            
           //fine cell adjustment 
              if(std::sin(X_apex(3, 0)) > 0.8 )
              {
                fine_num = -26;
                N=2; S=0; E=sag; W=0;
              }
              else if(std::sin(X_apex(3, 0)) < -0.8)
              {
                fine_num = 26;
                N=0; S=2; E=0; W=sag;
              }
              else if(std::cos(X_apex(3, 0)) < -0.8 )
              {
                fine_num = -1;
                N=sag; S=0; E=0; W=2;
              }     
              else if(std::cos(X_apex(3, 0)) > 0.8)
              {
                fine_num = 1;
                N=0; S=sag; E=2; W=0;
              } 

         }
          else if (l2 > 0.2)
        {
           X_d(0, 0) += s2_foot*std::cos(X_apex(3, 0)) -0.104*std::sin(X_apex(3, 0)); //X_apex(0, 0);
           X_d(1, 0) += s2_foot*std::sin(X_apex(3, 0)) +0.104*std::cos(X_apex(3, 0)); //X_apex(1, 0);
           X_d(2, 0) =  p_foot(2, 0) + prim(4, 0); 
           X_d(3, 0) += prim(1, 0);
           X_d(4, 0) = prim(3, 0);
            
           //fine cell adjustment 
              if(std::sin(X_apex(3, 0)) > 0.8 )
              {
                fine_num = -26;
                N=1; S=0; E=sag; W=0;
              }
              else if(std::sin(X_apex(3, 0)) < -0.8)
              {
                fine_num = 26;
                N=0; S=1; E=0; W=sag;
              }
              else if(std::cos(X_apex(3, 0)) < -0.8 )
              {
                fine_num = -1;
                N=sag; S=0; E=0; W=1;
              }     
              else if(std::cos(X_apex(3, 0)) > 0.8)
              {
                fine_num = 1;
                N=0; S=sag; E=1; W=0;
              } 

         }
         
         else if (l2 < 0)
         {
          if (l2 + 0.104)
          {
           X_d(0, 0) += s2_foot*std::cos(X_apex(3, 0)) +0.104*std::sin(X_apex(3, 0)); //X_apex(0, 0);
           X_d(1, 0) += s2_foot*std::sin(X_apex(3, 0)) -0.104*std::cos(X_apex(3, 0)); //X_apex(1, 0);
           X_d(2, 0) =  p_foot(2, 0) + prim(4, 0); 
           X_d(3, 0) += prim(1, 0);
           X_d(4, 0) = prim(3, 0);
          }
          else
          {
           X_d(0, 0) += s2_foot*std::cos(X_apex(3, 0)) +0.208*std::sin(X_apex(3, 0)); //X_apex(0, 0);
           X_d(1, 0) += s2_foot*std::sin(X_apex(3, 0)) -0.208*std::cos(X_apex(3, 0)); //X_apex(1, 0);
           X_d(2, 0) =  p_foot(2, 0) + prim(4, 0); 
           X_d(3, 0) += prim(1, 0);
           X_d(4, 0) = prim(3, 0);

          }
   
              if(std::sin(X_apex(3, 0)) > 0.8 )
              {
                fine_num = 26;
                N=0; S=1; E=sag; W=0;
              }
              else if(std::sin(X_apex(3, 0)) < -0.8)
              {
                fine_num = -26;
                N=1; S=0; E=0; W=sag;
              }
              else if(std::cos(X_apex(3, 0)) < -0.8 )
              {
                fine_num = 1;
                N=sag; S=0; E=1; W=0;
              }     
              else if(std::cos(X_apex(3, 0)) > 0.8)
              {
                fine_num = -1;
                N=0; S=sag; E=0; W=1;
              } 
         }
         
           else{
            X_d(0, 0) += s2_foot*std::cos(X_apex(3, 0)); //X_apex(0, 0);
            X_d(1, 0) += s2_foot*std::sin(X_apex(3, 0)); //X_apex(1, 0);
            X_d(2, 0) =  p_foot(2, 0) + prim(4, 0); 
            X_d(3, 0) += prim(1, 0);
            X_d(4, 0) = prim(3, 0);  
                    if(std::sin(X_apex(3, 0)) > 0.8 )
                    {
                       N=0;  S=0;  E=sag;  W=0;          
                    }
                    else if(std::sin(X_apex(3, 0)) < -0.8)
                    {
                       N=0;  S=0;  E=0;  W=sag;  
                    }
                    else if(std::cos(X_apex(3, 0)) < -0.8 )
                    {
                      N=sag;  S=0;  E=0;  W=0;  
                    }     
                    else if(std::cos(X_apex(3, 0)) > 0.8)
                    {
                      N=0;  S=sag;  E=0;  W=0;  
                    }      

           }
      }
      else
      {
          if (l2 < -0.314)
        {
          X_d(0, 0) += s2_foot*std::cos(X_apex(3, 0)) +0.208*std::sin(X_apex(3, 0)); //X_apex(0, 0);
          X_d(1, 0) += s2_foot*std::sin(X_apex(3, 0)) -0.208*std::cos(X_apex(3, 0)); //X_apex(1, 0);
          X_d(2, 0) =  p_foot(2, 0) + prim(4, 0); 
          X_d(3, 0) += prim(1, 0);
          X_d(4, 0) = prim(3, 0);
             // std::cout <<  "here 2 " << std::endl;
              if(std::sin(X_apex(3, 0)) > 0.8 )
              {
                fine_num = 26;
                N=0; S=2; E=sag; W=0;
              }
              else if(std::sin(X_apex(3, 0)) < -0.8)
              {
                fine_num = -26;
                N=2; S=0; E=0; W=sag;                
              }
              else if(std::cos(X_apex(3, 0)) < -0.8 )
              {
                fine_num = 1;
                N=sag; S=0; E=2; W=0;                
              }     
              else if(std::cos(X_apex(3, 0)) > 0.8)
              {
                fine_num = -1;
                N=0; S=sag; E=0; W=2;               
              } 



        }

         else if (l2 < -0.2)
        {
          X_d(0, 0) += s2_foot*std::cos(X_apex(3, 0)) +0.104*std::sin(X_apex(3, 0)); //X_apex(0, 0);
          X_d(1, 0) += s2_foot*std::sin(X_apex(3, 0)) -0.104*std::cos(X_apex(3, 0)); //X_apex(1, 0);
          X_d(2, 0) =  p_foot(2, 0) + prim(4, 0); 
          X_d(3, 0) += prim(1, 0);
          X_d(4, 0) = prim(3, 0);
           //   std::cout <<  "here 2 " << std::endl;
              if(std::sin(X_apex(3, 0)) > 0.8 )
              {
                fine_num = 26;
                N=0; S=1; E=sag; W=0;
              }
              else if(std::sin(X_apex(3, 0)) < -0.8)
              {
                fine_num = -26;
                N=1; S=0; E=0; W=sag;                
              }
              else if(std::cos(X_apex(3, 0)) < -0.8 )
              {
                fine_num = 1;
                N=sag; S=0; E=1; W=0;                
              }     
              else if(std::cos(X_apex(3, 0)) > 0.8)
              {
                fine_num = -1;
                N=0; S=sag; E=0; W=1;               
              } 



        }
        else if (l2 > 0)
        {
          X_d(0, 0) += s2_foot*std::cos(X_apex(3, 0)) -0.104*std::sin(X_apex(3, 0)); //X_apex(0, 0);
          X_d(1, 0) += s2_foot*std::sin(X_apex(3, 0)) +0.104*std::cos(X_apex(3, 0)); //X_apex(1, 0);
          X_d(2, 0) =  p_foot(2, 0) + prim(4, 0); 
          X_d(3, 0) += prim(1, 0);
          X_d(4, 0) = prim(3, 0);
             // std::cout <<  "here 22 " << std::endl;
              if(std::sin(X_apex(3, 0)) > 0.8 )
              {
                fine_num = -26;
                N=1; S=0; E=sag; W=0; 
              }
              else if(std::sin(X_apex(3, 0)) < -0.8)
              {
                fine_num = 26;
                N=0; S=1; E=0; W=sag; 
              }
              else if(std::cos(X_apex(3, 0)) < -0.8 )
              {
                fine_num = -1;
                N=sag; S=0; E=0; W=1; 
              }     
              else if(std::cos(X_apex(3, 0)) > 0.8)
              {
                fine_num = 1;
                N=0; S=sag; E=1; W=0;            
              } 
        }
        
        else{
          X_d(0, 0) += s2_foot*std::cos(X_apex(3, 0)); //X_apex(0, 0);
          X_d(1, 0) += s2_foot*std::sin(X_apex(3, 0)); //X_apex(1, 0);
          X_d(2, 0) =  p_foot(2, 0) + prim(4, 0); 
          X_d(3, 0) += prim(1, 0);
          X_d(4, 0) = prim(3, 0);
                    if(std::sin(X_apex(3, 0)) > 0.8 )
                    {
                       N=0;  S=0;  E=sag;  W=0;          
                    }
                    else if(std::sin(X_apex(3, 0)) < -0.8)
                    {
                      fine_num = -4;
                       N=0;  S=0;  E=0;  W=sag;  
                    }
                    else if(std::cos(X_apex(3, 0)) < -0.8 )
                    {
                      fine_num = -4*26;
                      N=sag;  S=0;  E=0;  W=0;  
                    }     
                    else if(std::cos(X_apex(3, 0)) > 0.8)
                    {
                      fine_num = 4*26;
                      N=0;  S=sag;  E=0;  W=0;  
                    }      
        }
      }
    } 
    else
    {
      X_d(0, 0) += s2_foot*std::cos(X_apex(3, 0)); //X_apex(0, 0);
      X_d(1, 0) += s2_foot*std::sin(X_apex(3, 0)); //X_apex(1, 0);
      X_d(2, 0) =  p_foot(2, 0) + prim(4, 0); 
      X_d(3, 0) += prim(1, 0);
      X_d(4, 0) = prim(3, 0);
                    if(std::sin(X_apex(3, 0)) > 0.8 )
                    {
                       N=0;  S=0;  E=sag;  W=0;          
                    }
                    else if(std::sin(X_apex(3, 0)) < -0.8)
                    {
                      fine_num = -4;
                       N=0;  S=0;  E=0;  W=sag;  
                    }
                    else if(std::cos(X_apex(3, 0)) < -0.8 )
                    {
                      fine_num = -4*26;
                      N=sag;  S=0;  E=0;  W=0;  
                    }     
                    else if(std::cos(X_apex(3, 0)) > 0.8)
                    {
                      fine_num = 4*26;
                      N=0;  S=sag;  E=0;  W=0;  
                    }      
    }



    waypoint(0, 0) += prim(0, 0)*std::cos(X_apex(3, 0));
    waypoint(1, 0) += prim(0, 0)*std::sin(X_apex(3, 0));
    waypoint(2, 0) =  p_foot(2, 0) + prim(4, 0); 
    waypoint(3, 0) += prim(1, 0);
    waypoint(4, 0) = prim(3, 0);

 
    // Record
    step += 1;
    if (stance == 0)
    {
      stance = 1;
    }
    else
    {
      stance = 0;
    }

      



    foot_dis = l2_foot - l2;
    // sim_list 
    Eigen::Matrix<double, 3, 1> sim;
    sim << l2, foot_dis, prim(1,0);
    sim_list.push_back(sim);
    //step lest
    Eigen::Matrix<double, 2, 1> step_sim;
    step_sim << std::abs(s2_foot-s1_foot),std::abs(l2_foot-l1_foot);
    step_list.push_back(step_sim);

    apex_list.push_back(X_apex);
    d_list.push_back(X_d);
    switch_list.push_back(X_switch);

    apextraj_list.push_back(apextraj);

    waypoint_list.push_back(waypoint);

    p_foot_list.push_back(p_foot);

    Eigen::Matrix<double, 3, 1> direction;
    direction << X_apex(3,0) - prim(1, 0),
                 X_apex(3,0) - prim(1, 0)*backward_num/(forward_num+backward_num),
                 X_apex(3,0);
    direction_list.push_back(direction);

    Eigen::Matrix<double, 2, 1> dt;
    dt << eps*forward_num, eps*backward_num;
    step_period.push_back(dt);
    Eigen::Matrix<double, 2, 1> slope;
    slope << aq * std::cos(X_apex(3, 0)), aq * std::sin(X_apex(3, 0));
    //slope << aq , bq ;
    step_surface.push_back(slope);

    Eigen::Matrix<double, 2, 1> WSQ;
    WSQ << w1_sq, w2_sq;
    WSQlist.push_back(WSQ);

        // step time, t1, t2
      Eigen::Matrix<double, 3, 1> time;
      time <<  eps*(forward_num+backward_num), eps*forward_num, eps*backward_num;
      Tlist.push_back(time);
  

  }

}  // namespace phase_space_planner