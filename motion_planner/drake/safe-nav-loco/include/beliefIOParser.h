#ifndef BELIEF_IO_PARSER_H
#define BELIEF_IO_PARSER_H

#include <string>
#include <iostream>
#include <fstream>

#include "drake/safe-nav-loco/include/json.hpp"
using json = nlohmann::json;

class BeliefIOParser
{
public:
    BeliefIOParser(std::string fileName);

    void setStep(int step);
    void advanceStep();
    int getProperty(std::string name);
    std::vector<int> getPropertyArray(std::string name);
    int getStep();

private:
    json jsonParser_;
    int stepCounter_;

};

#endif
