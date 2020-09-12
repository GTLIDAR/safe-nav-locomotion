#pragma once

#include <cmath>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <map>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>

#define PI 3.1416
#define G 9.81

typedef struct Action 
{
  double step_length;
  double dheading;
  double dheight;

  Action(const double _step_length = 0, const double _dheading = 0,
         const double _dheight = 0) 
  {
    step_length = _step_length;
    dheading = _dheading;
    dheight = _dheight;
  }

  bool operator<(const Action& a) const
  { 
    if ((std::fabs(this->step_length - a.step_length) < 0.0001) && 
        (std::fabs(this->dheading - a.dheading) < 0.0001) && 
        (std::fabs(this->dheight - a.dheight) < 0.0001))
    {
      return false;
    }
    else if ((std::fabs(this->step_length - a.step_length) < 0.0001) && 
             (std::fabs(this->dheading - a.dheading) < 0.0001))
    {
      return this->dheight < a.dheight;
    }
    else if (std::fabs(this->step_length - a.step_length) < 0.0001)
    {
      return this->dheading < a.dheading;
    }
    else
    {
      return this->step_length < a.step_length;
    }
  } 

} Action;

typedef struct State
{
  double v_apex;
  double h_apex;
  double foot_dis;

  State(const double _v_apex = 0.4, const double _h_apex = 0.8, const double _foot_dis = 0.07) 
  {
    v_apex = _v_apex;
    h_apex = _h_apex;
    foot_dis = _foot_dis;
  }

  bool operator<(const State& s) const
  { 
    if ((std::fabs(this->v_apex - s.v_apex) < 0.0001) && 
        (std::fabs(this->h_apex - s.h_apex) < 0.0001) && 
        (std::fabs(this->foot_dis - s.foot_dis) < 0.0001))
    {
      return false;
    }
    else if ((std::fabs(this->v_apex - s.v_apex) < 0.0001) && 
             (std::fabs(this->h_apex - s.h_apex) < 0.0001))
    {
      return this->foot_dis < s.foot_dis;
    }
    else if (std::fabs(this->v_apex - s.v_apex) < 0.0001)
    {
      return this->h_apex < s.h_apex;
    }
    else
    {
      return this->v_apex < s.v_apex;
    }
  } 

} State;

typedef struct Primitive 
{
  double step_length;
  double dheading;
  double dheight;
  double v_apex;
  double h_apex;
  Primitive(const double _step_length = 0, const double _dheading = 0,
            const double _dheight = 0, const double _v_apex = 0,
            const double _h_apex = 0.68) 
  {
    step_length = _step_length;
    dheading = _dheading;
    dheight = _dheight;
    v_apex = _v_apex;
    h_apex = _h_apex;
  }
} Primitive;

