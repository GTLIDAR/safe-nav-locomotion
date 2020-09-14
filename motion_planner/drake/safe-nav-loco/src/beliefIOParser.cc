#include "drake/safe-nav-loco/include/beliefIOParser.h"

BeliefIOParser::BeliefIOParser(std::string fileName)
{
    std::ifstream ifsParser(fileName.c_str());
    jsonParser_ = json::parse(ifsParser);
    stepCounter_ = 0;
    std::cout << "JSON imported" << std::endl;
}

void BeliefIOParser::advanceStep()
{
	stepCounter_++;
}

void BeliefIOParser::setStep(int step)
{
	stepCounter_ = step;
}

int BeliefIOParser::getProperty(std::string name)
{
	if(jsonParser_["saved_states"][std::to_string(stepCounter_)].contains(name))
	{
		return jsonParser_["saved_states"][std::to_string(stepCounter_)][name];
	}
	else
	{
		return jsonParser_["saved_states"][std::to_string(stepCounter_)]["action_info"]["State"][name];
	}
}

std::vector<int> BeliefIOParser::getPropertyArray(std::string name)
{
	if(jsonParser_["saved_states"][std::to_string(stepCounter_)].contains(name))
	{
		std::vector<int> ret = jsonParser_["saved_states"][std::to_string(stepCounter_)][name];
		return ret;
	}
	else
	{
		std::vector<int> ret = jsonParser_["saved_states"][std::to_string(stepCounter_)]["action_info"]["State"][name];
		return ret;
	}
}

int BeliefIOParser::getStep()
{
	return stepCounter_;
}


