#include "drake/safe-nav-loco/include/slugs_interface.h"

slugsInterface::slugsInterface(char* slugsLocation, char* inputFile)
{
    char *args[] = {slugsLocation, inputFile, "--interactiveStrategy", NULL};

    pid_ = 0;

    // Set up piping and stuff. Copied from stack overflow
    int inpipefd[2];
    int outpipefd[2];
    ret_code_ = pipe(inpipefd);
    ret_code_ = pipe(outpipefd);

    pid_=fork();

    if(pid_==0)
    {
        dup2(outpipefd[0], STDIN_FILENO);
        dup2(inpipefd[1], STDOUT_FILENO);
        dup2(inpipefd[1], STDERR_FILENO);

        //ask kernel to deliver SIGTERM in case the parent dies
        prctl(PR_SET_PDEATHSIG, SIGTERM);

        execv(args[0], args);

        exit(1);
    }

    close(outpipefd[0]);
    close(inpipefd[1]);
    // Thanks stack overflow

    // Reassign the pipes of interest
    outpipefd_ = outpipefd[1];
    inpipefd_ = inpipefd[0];

    //std::cout << "Waiting for initial output" << std::endl;
    std::cout << readAllOutputs() << std::endl;

    importInitialStates();
}

std::string slugsInterface::readAllOutputs()
{
    char buffer[256];
    std::string buffer_str("");
    ssize_t char_return = 1;
    do {
        char_return = read(inpipefd_, buffer, 255);
        buffer[char_return] = '\0';
        buffer_str += buffer;

    } while(buffer_str.find(">") == std::string::npos);

    if(buffer_str.c_str()[0] == '\n')
    {
        buffer_str = buffer_str.substr(1);
    }
    return buffer_str.substr(0, buffer_str.size() - 4);
}


void slugsInterface::importInitialStates()
{
    std::string getInputsStr = "XPRINTINPUTS\n";
    std::string getOutputsStr = "XPRINTOUTPUTS\n";
    std::string getInitialState = "XGETINIT\n";

    // std::cout << "Importing States" << std::endl;
    // First load the input stuff
    ret_code_ = write(outpipefd_, getInputsStr.c_str(), getInputsStr.size());
    std::string variables(readAllOutputs());
    std::istringstream f(variables);
    std::string line;
    while(std::getline(f, line))
    {
        if(line != "" && line.find(">") == std::string::npos)
        {
            input_vars_.push_back(line);
            variables_[line] = 0;
        }
    }

    // Get all output variables and populate data structures
    ret_code_ = write(outpipefd_, getOutputsStr.c_str(), getOutputsStr.size());
    variables = readAllOutputs();
    f = std::istringstream(variables);
    while(std::getline(f, line))
    {
        if(line != "" && line.find(">") == std::string::npos)
        {
            output_vars_.push_back(line);
            variables_[line] = 0;
        }
    }

    // Get initial state of everything
    ret_code_ = write(outpipefd_, getInitialState.c_str(), getInitialState.size());
    std::string initStates(readAllOutputs());
    unsigned int ii=0;
    while(ii < input_vars_.size())
    {
        variables_[input_vars_[ii]] = (initStates.c_str()[ii] == '1') ? true : false; 
        ii++;
    }  
    while(ii < input_vars_.size() + output_vars_.size())
    {
        variables_[output_vars_[ii - input_vars_.size()]] = (initStates.c_str()[ii] == '1') ? true : false; 
        ii++;
    }

}

void slugsInterface::setInput(std::string var_name, bool value)
{
    variables_[var_name] = value;
}

void slugsInterface::setInput(std::map<std::string, bool> input_map)
{
    for (std::map<std::string, bool>::iterator it=input_map.begin(); it!=input_map.end(); ++it)
    {
        variables_[it->first] = it->second; 
    }  
}

int slugsInterface::makeTransition()
{
    std::string message = "XMAKETRANS\n";
    ret_code_ = write(outpipefd_, message.c_str(), message.size());

    std::cout << "Input variables" << std::endl;
    message = "";
    // std::cout << input_vars_.size() << std::endl;
    for(uint ii=0; ii < input_vars_.size(); ii++)
    {

        // std::cout << variables_[input_vars_[ii]] << std::endl;
        // Convert bools to string
        message += variables_[input_vars_[ii]] ? '1': '0';
    }
    message+='\n';
    std::cout << message;
    ret_code_ = write(outpipefd_, message.c_str(), message.size());

    // Update outputs based on return
    std::string return_values(readAllOutputs());
    std::cout << "Output variables" << std::endl;
    std::cout << return_values << std::endl;
    
    if(return_values.find("ERROR") == std::string::npos)
    {
        unsigned int ii=0;
        while(ii < input_vars_.size())
        {
            variables_[input_vars_[ii]] = (return_values.c_str()[ii] == '1') ? true : false; 
            ii++;
        }  
        while(ii < input_vars_.size() + output_vars_.size())
        {
            variables_[output_vars_[ii - input_vars_.size()]] = (return_values.c_str()[ii] == '1') ? true : false; 
            ii++;
        }
        return 1; 
    }
    else
    {
        return 0;
    }
}


bool slugsInterface::getValue(std::string var_name)
{
    return variables_[var_name];
}

std::vector<std::string> slugsInterface::getInputNames()
{
    return input_vars_;
}

std::vector<std::string> slugsInterface::getOutputNames()
{
    return output_vars_;
}

std::map<std::string, bool> slugsInterface::getSystemState()
{
    return variables_;
}

void slugsInterface::printStates()
{
    std::cout << "Inputs" << std::endl;
    for(uint ii=0; ii < input_vars_.size(); ii++)
    {
        std::cout << input_vars_[ii] << ": " << getValue(input_vars_[ii]) << std::endl;
    }
    std::cout << "\nOutputs" << std::endl;
    for(uint ii=0; ii < output_vars_.size(); ii++)
    {
        std::cout << output_vars_[ii] << ": " << getValue(output_vars_[ii]) << std::endl;
    }
}
