#include <iostream>
#include <Eigen/Dense>
#include "CassieIK.h"

int main(int argc, char** argv){
    Cassie myObj;
    myObj.COM << -2.70000000, 4.20000000, 0.80000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000;
    myObj.l_foot << -2.69999927, 4.10000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000;
    myObj.r_foot << -2.70000073, 4.30000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000;
    myObj.heading << 3.1416;
    std::cout << myObj.DoIK();
    return 0; 
}