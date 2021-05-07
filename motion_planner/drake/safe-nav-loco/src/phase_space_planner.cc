#include "drake/safe-nav-loco/include/phase_space_planner.h"


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
                
                //prim(3, 0) = good_vn[(sweep_c/2)];
               /*
                if(sweep_c == 0)
                {
                  prim(3, 0) = 0.2;
                  //goodlateral(0,0) = 0;
                  if (l2 > 0)
                  {
                  //COS = std::cos(X_apex(3, 0) + prim(1, 0)-0.5*std::abs((0.2 - l2)));
                  //SIN = std::sin(X_apex(3, 0) + prim(1, 0)-0.5*std::abs((0.2 - l2)));
                    phi = 0;//-0.1 - waypoint(3,0);
                    //prim(1, 0)=prim(1,0)+phi;
                    COS = std::cos(X_apex(3, 0) + prim(1, 0)+phi);
                    SIN = std::sin(X_apex(3, 0) + prim(1, 0)+phi);
                  }
                  else{
                  //COS = std::cos(X_apex(3, 0) + prim(1, 0)+0.5*std::abs((0.2 - l2)));
                  //SIN = std::sin(X_apex(3, 0) + prim(1, 0)+0.5*std::abs((0.2 - l2)));
                   phi = 0;//0.1 - waypoint(3,0);
                    //prim(1, 0)=prim(1,0)+phi;
                    COS = std::cos(X_apex(3, 0) + prim(1, 0)+phi);
                    SIN = std::sin(X_apex(3, 0) + prim(1, 0)+phi);
                  }
                  

                }
                */
               // else{
                 // if (goodlateral(0,0) == 0)
                 // {
                 //    prim(3, 0) = good_vn[0];
                 //     std::cout << "prev is bad lateral"   << std::endl;
                      //phi = 0;
                 // }
                 // else
                 // {
                  prim(3, 0) = good_vn[((sweep_c-1)/2)];
                  prim(3, 0) = opt_vn;
                  //std::cout << "prev is good lateral"   << std::endl;
                  //goodlateral(0,0) = 0;
                  //phi = 0;
                 //} 
                 //goodlateral(0,0) = 1;
                  phi = 0 ;//- waypoint(3,0);
                  
                // }

                //prim(3, 0) = sum_vn/sweep_c;
                //std::cout <<  "availble vn= "<< good_vn[0] << ", " << good_vn[sweep_c-1]<< std::endl;
                //std::cout <<  "used vn= "<< prim (3, 0) << std::endl;
                //std::cout << "good_vn size= "<< sweep_c-1   << std::endl;
                //std::cout <<  "phi= "<< phi << std::endl;
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
           /* 
            if (prim(0, 0 ) < 0.4)
            {
                  if (l2 < 0)
                {
                  good_vn[sweep_c]=prim(3, 0);
                  //std::cout << "good_vn= "<< good_vn[sweep_c]<< std::endl;
                  sweep_c= sweep_c+1;
                  sum_vn = sum_vn + prim(3,0);
                  //std::cout << "I am here"<< std::endl;

                  new_cost = std::abs((-0.1-l2)) + 5*std::abs((-0.135-(l2_foot - l2)));  //delta_y1 + delta_y2
                    //std::cout << "vn= "<< prim(3, 0) << std::endl;
                    //std::cout << "new_cost= "<< new_cost << std::endl;   
                  if(new_cost < cost)
                  {
                    cost = new_cost;
                    opt_vn = prim(3, 0);
                   // std::cout << "opt_vn= "<< opt_vn << std::endl;
                   // std::cout << "cost= "<< cost << std::endl;                    
                  }



                }
            }
            */

          // else{
            if (prim(1, 0) == 0)
            {
                //    if ((l2 < 0)  && (std::abs(l2_foot-l2) < 0.3) && (l2_foot > -0.5))
                 //   {
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
                      
                  // }

            }
            else{

                //    if ((l2 < 0)  && (std::abs(l2_foot-l2) < 0.3) && (l2_foot > -0.5))
                 //   {
                      good_vn[sweep_c]=prim(3, 0);
                      //std::cout << "good_vn= "<< good_vn[sweep_c]<< std::endl;
                      sweep_c= sweep_c+1;
                      sum_vn = sum_vn + prim(3,0);

                      new_cost = 7*std::abs((l2)) + 4*std::abs((-0.075-(l2_foot - l2)));  //delta_y1 + delta_y2
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
                      
                  // }

            }

   
               // }

           }
           else
           {
             /* 
              if (prim(0, 0 ) < 0.4)
              {
                if (l2 > 0)
                  {
                    good_vn[sweep_c]=prim(3, 0);
                    //std::cout << "good_vn= "<< good_vn[sweep_c]<< std::endl;
                    sweep_c= sweep_c + 1;
                    sum_vn = sum_vn + prim(3,0);
                    //std::cout << "I am here"<< std::endl;
                    new_cost = std::abs((0.1-l2)) + 5*std::abs((0.135-(l2_foot - l2)));  //delta_y1 + delta_y2
                    //std::cout << "vn= "<< prim(3, 0) << std::endl;
                    //std::cout << "new_cost= "<< new_cost << std::endl;                       
                    if(new_cost < cost)
                    {
                      cost = new_cost;
                      opt_vn = prim(3, 0);
                     // std::cout << "opt_vn= "<< opt_vn << std::endl;
                     // std::cout << "cost= "<< cost << std::endl;                      
                    }

                  }

              }
              */
             //else
             // {
            if(prim(1, 0)==0)
            {
                               // if ((l2 > 0) && (std::abs(l2_foot-l2) < 0.3) && (l2_foot < 0.5))
                 // {
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

                 // }


            }
            else
            {

                    // if ((l2 > 0) && (std::abs(l2_foot-l2) < 0.3) && (l2_foot < 0.5))
                 // {
                    good_vn[sweep_c]=prim(3, 0);
                    //std::cout << "good_vn= "<< good_vn[sweep_c]<< std::endl;
                    sweep_c= sweep_c + 1;
                    sum_vn = sum_vn + prim(3,0);
                    //std::cout << "sweep_c= "<< sweep_c << std::endl;
                    new_cost = 7*std::abs((l2)) + 4*std::abs((0.075-(l2_foot - l2)));  //delta_y1 + delta_y2
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

                 // }


            }

              // }
           }
        


        }
        //std::cout << "dy1_n= " << l2 << ", l2_foot= "<< l2_foot << std::endl ;
        //std::cout <<  "phi= "<< phi << std::endl;
        std::cout <<  "step length= "<< prim(0,0) << std::endl;
        std::cout <<  "used vn= "<< prim (3, 0) << std::endl << std::endl << std::endl;

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

 
  
    X_d(0, 0) += prim(0, 0)*std::cos(X_apex(3, 0)); //X_apex(0, 0);//prim(0, 0)*std::cos(X_apex(3, 0));
    X_d(1, 0) += prim(0, 0)*std::sin(X_apex(3, 0)); //X_apex(1, 0) ;//prim(0, 0)*std::sin(X_apex(3, 0));
    X_d(2, 0) =  p_foot(2, 0) + prim(4, 0); 
    X_d(3, 0) += prim(1, 0);
    X_d(4, 0) = prim(3, 0);

    waypoint(0, 0) += prim(0, 0)*std::cos(X_apex(3, 0)+phi);
    waypoint(1, 0) += prim(0, 0)*std::sin(X_apex(3, 0)+phi);
    waypoint(2, 0) =  p_foot(2, 0) + prim(4, 0); 
    waypoint(3, 0) += phi;
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
    //Tlist.push_back(TS);
    //std::cout << "num= " << num << std::endl;

 

    Eigen::Matrix<double, 6, 1> start = apextraj_list[step-1];//apextraj_list[step-1];//switch_list[step-2];
    Eigen::Matrix<double, 6, 1> end = switch_list[step-1];//apextraj_list[step];//switch_list[step-1];

    Eigen::Matrix<double, 3, 1> pre_foot = p_foot_list[step-2];
    Eigen::Matrix<double, 3, 1> cur_foot = p_foot_list[step-1];
    Eigen::Matrix<double, 3, 1> nex_foot = p_foot_list[step];

    Eigen::Matrix<double, 2, 1> slope1 = step_surface[step-2];
    Eigen::Matrix<double, 2, 1> slope2 = step_surface[step-1];

    Eigen::Matrix<double, 2, 1> wwsq = WSQlist[step-2];

    double x_ddot, x_dot = start(3, 0), x = start(0, 0);
    double y_ddot, y_dot = start(4, 0), y = start(1, 0);
    double z_ddot, z_dot = start(5, 0), z = start(2, 0);


    double x_swing, dx_swing, ddx_swing;
    double y_swing, dy_swing, ddy_swing;
    double z_swing, dz_swing, ddz_swing;

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
    int num1 = t1 / dt;
    int num2 = t2 / dt;
    int num3 = ts / dt;

    int numtt= (tt1+tt2)/dt;
    int num11= (tt1)/dt;

    double w_sq = wwsq(1, 0);
    
    double inc_x, inc_y;


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
      }

      Eigen::Matrix<double, 10, 1> COM;
      COM << x, y, z, x_dot, y_dot, z_dot, x_ddot, y_ddot, z_ddot, d_t;
      COM_list.push_back(COM);

      x_swing = pre_foot(0, 0) + x_distance / 2 * (1 - std::cos(PI * i / num));
      dx_swing = x_distance / 2 * std::sin(PI * i / num) * PI / T;
      ddx_swing = x_distance / 2 * std::cos(PI * i / num) * std::pow(PI / T, 2);

      y_swing = pre_foot(1, 0) + y_distance / 2 * (1 - std::cos(PI * i / num));
      dy_swing = y_distance / 2 * std::sin(PI * i / num) * PI / T;
      ddy_swing = y_distance / 2 * std::cos(PI * i / num) * std::pow(PI / T, 2);


      if(i < num1)
      {
        z_swing = pre_foot(2, 0) + h1 / 2 * (1 - std::cos(PI * i / num1));
        dz_swing = h1 / 2 * std::sin(PI * i / num1) * PI / t1;
        ddz_swing = h1 / 2 * std::cos(PI * i / num1) * std::pow(PI / t1, 2);
      }
      else if (i < num1 + num3)
      {
        z_swing = pre_foot(2, 0) + h1;
        dz_swing = 0;
        ddz_swing = 0;
      }
      else
      {
        z_swing = nex_foot(2, 0) + h2 / 2 * (1 + std::cos(PI * (i-num1-num3) / num2));
        dz_swing = -h2 / 2 * std::sin(PI * (i-num1-num3) / num2) * PI / t2;
        ddz_swing = -h2 / 2 * std::cos(PI * (i-num1-num3) / num2) * std::pow(PI / t2, 2);
      }

      Eigen::Matrix<double, 10, 1> swing_foot;
      swing_foot << x_swing, y_swing, z_swing, dx_swing, dy_swing, dz_swing, 
                    ddx_swing, ddy_swing, ddz_swing, d_t;
      Eigen::Matrix<double, 10, 1> stance_foot;
      stance_foot << cur_foot(0, 0), cur_foot(1, 0), cur_foot(2, 0), 0, 0, 0, 0, 0, 0, d_t;

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
    //double tt1=step_period[step-1](0, 0);
    //double tt2=step_period[step-1](1, 0);
    

    Eigen::Matrix<double, 1, 1> TS;
    Tstep = Tstep + T;
    TS << Tstep;
    //Tlist.push_back(TS);
    
    Eigen::Matrix<double, 6, 1> start;
    start << apex_list[step-1](0, 0), apex_list[step-1](1, 0), apex_list[step-1](2, 0), 
             apex_list[step-1](4, 0)*cos(apex_list[step-1](3, 0)), 
             apex_list[step-1](4, 0)*sin(apex_list[step-1](3, 0)), 0;
             
    //Eigen::Matrix<double, 6, 1> start = apextraj_list[step-1];
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

    double x_swing, dx_swing, ddx_swing;
    double y_swing, dy_swing, ddy_swing;
    double z_swing, dz_swing, ddz_swing;

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

    int num1 = t1 / dt;
    int num2 = t2 / dt;
    int num3 = ts / dt;

    double w_sq = wwsq(1, 0); 
    
    double inc_x, inc_y;
    //int numtt= (tt1+tt2)/dt;
    //int num11= (tt1)/dt;
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
      heading << (direction_list[step-1](1, 0) - direction_list[step-1](0, 0))
                  *(i  + 1)/num + direction_list[step-1](0, 0);
      heading_list.push_back(heading);

      x_swing = pre_foot(0, 0) + x_distance / 2 * (1 - std::cos(PI * i / num));
      dx_swing = x_distance / 2 * std::sin(PI * i / num) * PI / T;
      ddx_swing = x_distance / 2 * std::cos(PI * i / num) * std::pow(PI / T, 2);

      y_swing = pre_foot(1, 0) + y_distance / 2 * (1 - std::cos(PI * i / num));
      dy_swing = y_distance / 2 * std::sin(PI * i / num) * PI / T;
      ddy_swing = y_distance / 2 * std::cos(PI * i / num) * std::pow(PI / T, 2);



      if(i < num1)
      {
        z_swing = pre_foot(2, 0) + h1 / 2 * (1 - std::cos(PI * i / num1));
        dz_swing = h1 / 2 * std::sin(PI * i / num1) * PI / t1;
        ddz_swing = h1 / 2 * std::cos(PI * i / num1) * std::pow(PI / t1, 2);
      }
      else if (i < num1 + num3)
      {
        z_swing = pre_foot(2, 0) + h1;
        dz_swing = 0;
        ddz_swing = 0;
      }
      else
      {
        z_swing = nex_foot(2, 0) + h2 / 2 * (1 + std::cos(PI * (i-num1-num3) / num2));
        dz_swing = -h2 / 2 * std::sin(PI * (i-num1-num3) / num2) * PI / t2;
        ddz_swing = -h2 / 2 * std::cos(PI * (i-num1-num3) / num2) * std::pow(PI / t2, 2);
      }

      Eigen::Matrix<double, 10, 1> swing_foot;
      swing_foot << x_swing, y_swing, z_swing, dx_swing, dy_swing, dz_swing, 
                    ddx_swing, ddy_swing, ddz_swing, d_t;
      Eigen::Matrix<double, 10, 1> stance_foot;
      stance_foot << cur_foot(0, 0), cur_foot(1, 0), cur_foot(2, 0), 0, 0, 0, 0, 0, 0, d_t;

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

    double x_swing, dx_swing, ddx_swing;
    double y_swing, dy_swing, ddy_swing;
    double z_swing, dz_swing, ddz_swing;

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

      x_swing = pre_foot(0, 0) + x_distance / 2 * (1 - std::cos(PI * i / num));
      dx_swing = x_distance / 2 * std::sin(PI * i / num) * PI / T;
      ddx_swing = x_distance / 2 * std::cos(PI * i / num) * std::pow(PI / T, 2);

      y_swing = pre_foot(1, 0) + y_distance / 2 * (1 - std::cos(PI * i / num));
      dy_swing = y_distance / 2 * std::sin(PI * i / num) * PI / T;
      ddy_swing = y_distance / 2 * std::cos(PI * i / num) * std::pow(PI / T, 2);



      if(i < num1)
      {
        z_swing = pre_foot(2, 0) + h1 / 2 * (1 - std::cos(PI * i / num1));
        dz_swing = h1 / 2 * std::sin(PI * i / num1) * PI / t1;
        ddz_swing = h1 / 2 * std::cos(PI * i / num1) * std::pow(PI / t1, 2);
      }
      else if (i < num1 + num3)
      {
        z_swing = pre_foot(2, 0) + h1;
        dz_swing = 0;
        ddz_swing = 0;
      }
      else
      {
        z_swing = nex_foot(2, 0) + h2 / 2 * (1 + std::cos(PI * (i-num1-num3) / num2));
        dz_swing = -h2 / 2 * std::sin(PI * (i-num1-num3) / num2) * PI / t2;
        ddz_swing = -h2 / 2 * std::cos(PI * (i-num1-num3) / num2) * std::pow(PI / t2, 2);
      }

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


    void PhaseSpacePlanner::side_step()
  {
  
    double T = 0.2;
    double dt = 0.001; 
    int num = T/dt;

    Eigen::Matrix<double, 1, 1> TS;
    Tstep = Tstep + T;
    TS << Tstep;
    //Tlist.push_back(TS);
    //std::cout << "num= " << num << std::endl;

    

    Eigen::Matrix<double, 6, 1> start = switch_list[step-2];
    Eigen::Matrix<double, 6, 1> end = switch_list[step-1];

    Eigen::Matrix<double, 3, 1> pre_foot = p_foot_list[step-2];
    Eigen::Matrix<double, 3, 1> cur_foot = p_foot_list[step-1];
    Eigen::Matrix<double, 3, 1> nex_foot;

        if (stance == 0)
    {    
      nex_foot << d_list[step](0, 0) + 0.135 * std::sin(-apex_list[step](3, 0)),
                  d_list[step](1, 0) + 0.135 * std::cos(-apex_list[step](3, 0)), 
                  p_foot_list[step](2, 0);
    }
    else
    {
      nex_foot << d_list[step](0, 0) - 0.135 * std::sin(-apex_list[step](3, 0)),
                  d_list[step](1, 0) - 0.135 * std::cos(-apex_list[step](3, 0)), 
                  p_foot_list[step](2, 0);
    }

    Eigen::Matrix<double, 2, 1> slope1 = step_surface[step-2];
    Eigen::Matrix<double, 2, 1> slope2 = step_surface[step-1];

    Eigen::Matrix<double, 2, 1> wwsq = WSQlist[step-2];

    double x_ddot, x_dot = start(3, 0), x = start(0, 0);
    double y_ddot, y_dot = start(4, 0), y = start(1, 0);
    double z_ddot, z_dot = start(5, 0), z = start(2, 0);

    double x_swing, dx_swing, ddx_swing;
    double y_swing, dy_swing, ddy_swing;
    double z_swing, dz_swing, ddz_swing;

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

    int num0 = step_period[step-2](1, 0) / dt;
    int num1 = t1 / dt;
    int num2 = t2 / dt;
    int num3 = ts / dt;

    double w_sq = wwsq(1, 0);
    
    double inc_x, inc_y;


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

      if (i < num0)
      {
        z_ddot = slope1(0, 0) * x_ddot + slope1(1, 0) * y_ddot;
        z = z + slope1(0, 0) * inc_x + slope1(1, 0) * inc_y;
        z_dot = slope1(0, 0) * x_dot + slope1(1, 0) * y_dot;

        Eigen::Matrix<double, 1, 1> heading;
        heading << (direction_list[step-2](2, 0) - direction_list[step-2](1, 0))
                  *(i + 1)/num0 
                  + direction_list[step-2](1, 0);
        heading_list.push_back(heading);
      }
      else
      {
        z_ddot = slope2(0, 0) * x_ddot + slope2(1, 0) * y_ddot;
        z = z + slope2(0, 0) * inc_x + slope2(1, 0) * inc_y;
        z_dot = slope2(0, 0) * x_dot + slope2(1, 0) * y_dot;

        Eigen::Matrix<double, 1, 1> heading;
        heading << (direction_list[step-1](1, 0) - direction_list[step-1](0, 0))
                  *(i  + 1 - num0)/(num - num0)
                  + direction_list[step-1](0, 0);
        heading_list.push_back(heading);
      }

      Eigen::Matrix<double, 10, 1> COM;
      COM << x, y, z, x_dot, y_dot, z_dot, x_ddot, y_ddot, z_ddot, d_t;
      COM_list.push_back(COM);

      x_swing = pre_foot(0, 0) + x_distance / 2 * (1 - std::cos(PI * i / num));
      dx_swing = x_distance / 2 * std::sin(PI * i / num) * PI / T;
      ddx_swing = x_distance / 2 * std::cos(PI * i / num) * std::pow(PI / T, 2);

      y_swing = pre_foot(1, 0) + y_distance / 2 * (1 - std::cos(PI * i / num));
      dy_swing = y_distance / 2 * std::sin(PI * i / num) * PI / T;
      ddy_swing = y_distance / 2 * std::cos(PI * i / num) * std::pow(PI / T, 2);


      if(i < num1)
      {
        z_swing = pre_foot(2, 0) + h1 / 2 * (1 - std::cos(PI * i / num1));
        dz_swing = h1 / 2 * std::sin(PI * i / num1) * PI / t1;
        ddz_swing = h1 / 2 * std::cos(PI * i / num1) * std::pow(PI / t1, 2);
      }
      else if (i < num1 + num3)
      {
        z_swing = pre_foot(2, 0) + h1;
        dz_swing = 0;
        ddz_swing = 0;
      }
      else
      {
        z_swing = nex_foot(2, 0) + h2 / 2 * (1 + std::cos(PI * (i-num1-num3) / num2));
        dz_swing = -h2 / 2 * std::sin(PI * (i-num1-num3) / num2) * PI / t2;
        ddz_swing = -h2 / 2 * std::cos(PI * (i-num1-num3) / num2) * std::pow(PI / t2, 2);
      }

      Eigen::Matrix<double, 10, 1> swing_foot;
      swing_foot << x_swing, y_swing, z_swing, dx_swing, dy_swing, dz_swing, 
                    ddx_swing, ddy_swing, ddz_swing, d_t;
      Eigen::Matrix<double, 10, 1> stance_foot;
      stance_foot << cur_foot(0, 0), cur_foot(1, 0), cur_foot(2, 0), 0, 0, 0, 0, 0, 0, d_t;

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
    
  }


  

}  // namespace phase_space_planner