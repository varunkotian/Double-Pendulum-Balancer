#include "MPC_Controller.h"
#include <cmath>
#include <algorithm>

MPC_Controller::MPC_Controller(DoublePendulum* pend, int horizon)
    : pendulum(pend), prediction_horizon(horizon)
{
}

double MPC_Controller::computeControl(const State& state)
{
    return optimizeControl(state);
}

double MPC_Controller::simulateAndComputeCost(const State& current_state, double torque)
{
    State sim_state = current_state;
    double total_cost = 0.0;
    
    for (int i = 0; i < prediction_horizon; ++i)
    {
        // Cost for state deviation from target (upright position)
        // double angle_cost = Q_angle * (sim_state.theta1 * sim_state.theta1 + sim_state.theta2 * sim_state.theta2);
        double angle_cost = Q_angle * ((1 - cos(sim_state.theta1)) + (1 - cos(sim_state.theta2)));
        double vel_cost = Q_angular_vel * (sim_state.theta1_dot * sim_state.theta1_dot + sim_state.theta2_dot * sim_state.theta2_dot);
        double control_cost = R * torque * torque;
        
        total_cost += angle_cost + vel_cost + control_cost;
        
        // Simulate one step
        DoublePendulum temp_pend;
        temp_pend.setState(sim_state);
        temp_pend.L1 = pendulum->L1;
        temp_pend.L2 = pendulum->L2;
        temp_pend.m1 = pendulum->m1;
        temp_pend.m2 = pendulum->m2;
        temp_pend.g = pendulum->g;
        temp_pend.b1 = pendulum->b1;
        temp_pend.b2 = pendulum->b2;
        
        temp_pend.update(time_step, torque);
        sim_state = temp_pend.getState();
    }
    
    return total_cost;
}

double MPC_Controller::optimizeControl(const State& state)
{
    // Grid search over torque values
    double best_torque = 0.0;
    double best_cost = simulateAndComputeCost(state, 0.0);
    
    double step = max_torque / 100.0;
    for (double torque = -max_torque; torque <= max_torque; torque += step)
    {
        double cost = simulateAndComputeCost(state, torque);
        if (cost < best_cost)
        {
            best_cost = cost;
            best_torque = torque;
        }
    }
    
    // Fine-tuned search around best torque
    step = step / 5.0;
    for (double torque = best_torque - 2*step; torque <= best_torque + 2*step; torque += step)
    {
        torque = std::max(-max_torque, std::min(max_torque, torque));
        double cost = simulateAndComputeCost(state, torque);
        if (cost < best_cost)
        {
            best_cost = cost;
            best_torque = torque;
        }
    }
    
    return best_torque;
}
