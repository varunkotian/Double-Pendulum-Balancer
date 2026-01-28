#ifndef MPC_CONTROLLER_H
#define MPC_CONTROLLER_H

#include "DoublePendulum.h"

class MPC_Controller
{
public:
    MPC_Controller(DoublePendulum* pendulum, int horizon = 200);
    
    double computeControl(const State& state);
    double simulateAndComputeCost(const State& state, double torque);
    
    // Parameters
    int prediction_horizon;
    double time_step = 0.01;
    
    // Cost function weights
    double Q_angle = 100.0;      // Weight for angle errors
    double Q_angular_vel = 10.0; // Weight for angular velocity errors
    double R = 0.1;              // Weight for control effort
    
    // Control constraints
    double max_torque = 100.0;

private:
    DoublePendulum* pendulum;
    
    // Optimize control input using gradient descent or similar
    double optimizeControl(const State& state);
};

#endif // MPC_CONTROLLER_H
