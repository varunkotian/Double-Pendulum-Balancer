#ifndef CONSOLE_OUTPUT_H
#define CONSOLE_OUTPUT_H

#include "DoublePendulum.h"

class ConsoleOutput
{
public:
    ConsoleOutput();
    
    void displayState(const DoublePendulum* pendulum, double control_torque, double mpc_cost, double time);
    void displayHeader();
    
private:
    int frame_count;
};

#endif // CONSOLE_OUTPUT_H
