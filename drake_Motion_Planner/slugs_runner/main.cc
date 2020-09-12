#include "drake/slugs_runner/slugs_interface.h"

int main(int argc, char* argv[])
{
    if(argc != 2)
    {
        std::cout << "Please provide 1 .slugsin file argument" << std::endl;
        return -1;
    }
    // Location of the slugs executable
    char * slugsLoc = "/home/yingke/slugs-master/src/slugs";

    slugsInterface slInt(slugsLoc, argv[1]);

    std::cout << "Moving states" << std::endl;
    slInt.setInput("car2", true);
    if(slInt.makeTransition())
    {
        slInt.printStates();
        std::cout << "It's a thing!" << std::endl;
    }
    else
    {
        std::cout << "Invalid Transition" << std::endl;
    }

    return 0;
}

