#ifndef SLUGS_INTERFACE_HPP
#define SLUGS_INTERFACE_HPP

#include <vector>
#include <string>
#include <iostream>
#include <map>
#include <sstream>

// Process stuff
#include <unistd.h>
#include <sys/prctl.h>
#include <signal.h>
#include <poll.h>
#include <fcntl.h>
#include <limits>
#include <sys/ioctl.h>

/**
XMAKETRANS - make a transition by inputting state variables followed by new line character in order
XPRINTINPUTS - get list of inputs
XPRINTOUTPUTS - get list of outputs
XGETINIT - get initial state
*/

/**
 * TODO:
 * prettyprint of stuff
 * maybe some more of the commands
 */

class slugsInterface
{
public:
    /**
     * @brief      Constructs the object.
     *
     * @param      slugsLocation  The slugs file location in your computer path
     * @param      inputFile      The .slugsin file to run over
     */
    slugsInterface(char* slugsLocation, char* inputFile);


    /**
     * @brief      Sets the value of an input variable
     *
     * @param[in]  var_name  The variable name 
     * @param[in]  value     The value
     * WARNING: Does not do any sort of error checking so please pass in a valid input
     * There is no reason to try to set an output variable because it will not change the automata
     */
    void setInput(std::string var_name, bool value);
    
    /**
     * @brief      Sets the input.
     *
     * @param[in]  input_map  The input map
     */
    void setInput(std::map<std::string, bool> input_map);

    /**
     * @brief      Based on the values currently in the input list, make a transition in the automata and update outputs
     */
    int makeTransition();

    
    // Getters
    
    /**
     * @brief      Get vector of input strings.
     *
     * @return     The input names.
     */
    std::vector<std::string> getInputNames();
   
    /**
     * @brief      Get vector of output strings
     *
     * @return     The output names.
     */
    std::vector<std::string> getOutputNames();

    /**
     * @brief      Get the value of some variable
     *
     * @param[in]  var_name  The variable name
     *
     * @return     The value.
     */
    bool getValue(std::string var_name);


    std::map<std::string, bool> getSystemState();

    void printStates();

private:
    pid_t pid_;
    int inpipefd_;
    int outpipefd_;
    int ret_code_; // because bazel gets mad if you ignore return statements

    std::vector<std::string> input_vars_;
    std::vector<std::string> output_vars_;

    // C++ maps are ordered
    std::map<std::string, bool> variables_;

    void importInitialStates();

    /**
     * @brief      Helper function to read the std::out of slugs
     *
     * @return     A string containing a slightly processed version of the slugs output.
     *             The '>' will be ignored as well as training new lines and initial new line
     *             Note that this function is a blocking read. Be careful when using it.
     */
    std::string readAllOutputs();

};




#endif
