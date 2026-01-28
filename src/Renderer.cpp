#include "Renderer.h"
#include <iostream>
#include <iomanip>
#include <cmath>

ConsoleOutput::ConsoleOutput() : frame_count(0)
{
}

void ConsoleOutput::displayHeader()
{
    std::cout << "\n";
    std::cout << "╔════════════════════════════════════════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║                         MPC Double Pendulum Balancer - Control Loop                               ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════════════════════════════════════════════╝\n";
    std::cout << std::left 
              << std::setw(8) << "Time(s)"
              << std::setw(12) << "θ1(rad)"
              << std::setw(12) << "θ1_dot"
              << std::setw(12) << "θ2(rad)"
              << std::setw(12) << "θ2_dot"
              << std::setw(15) << "Torque(N·m)"
              << std::setw(15) << "MPC_Cost"
              << "\n";
    std::cout << std::string(100, '-') << "\n";
}

void ConsoleOutput::displayState(const DoublePendulum* pendulum, double control_torque, double mpc_cost, double time)
{
    State state = pendulum->getState();
    
    std::cout << std::left << std::fixed << std::setprecision(4)
              << std::setw(8) << time
              << std::setw(12) << state.theta1
              << std::setw(12) << state.theta1_dot
              << std::setw(12) << state.theta2
              << std::setw(12) << state.theta2_dot
              << std::setw(15) << control_torque
              << std::setw(15) << mpc_cost
              << "\n";
    
    frame_count++;
}
