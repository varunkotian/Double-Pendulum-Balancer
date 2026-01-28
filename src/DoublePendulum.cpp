#define _USE_MATH_DEFINES
#include "DoublePendulum.h"
#include <cmath>
#include <algorithm>

DoublePendulum::DoublePendulum()
{
    // Initialize with pendulum at bottom position
    state = State(M_PI, 0.0, M_PI, 0.0);
}

void DoublePendulum::setState(const State& new_state)
{
    state = new_state;
}

State DoublePendulum::getState() const
{
    return state;
}

void DoublePendulum::update(double dt, double torque)
{
    // Clamp torque to reasonable bounds
    double clamped_torque = std::max(-10.0, std::min(10.0, torque));
    
    State curr_state = state;  // Save current state
    
    // RK4 integration
    State k1, k2, k3, k4;
    
    // k1
    double a1, a2;
    computeAccelerations(clamped_torque, a1, a2);
    k1 = State(curr_state.theta1_dot, a1, curr_state.theta2_dot, a2);
    
    // k2
    State temp_state = State(
        curr_state.theta1 + 0.5 * dt * k1.theta1,
        curr_state.theta1_dot + 0.5 * dt * k1.theta1_dot,
        curr_state.theta2 + 0.5 * dt * k1.theta2,
        curr_state.theta2_dot + 0.5 * dt * k1.theta2_dot
    );
    state = temp_state;
    computeAccelerations(clamped_torque, a1, a2);
    k2 = State(temp_state.theta1_dot, a1, temp_state.theta2_dot, a2);
    
    // k3
    temp_state = State(
        curr_state.theta1 + 0.5 * dt * k2.theta1,
        curr_state.theta1_dot + 0.5 * dt * k2.theta1_dot,
        curr_state.theta2 + 0.5 * dt * k2.theta2,
        curr_state.theta2_dot + 0.5 * dt * k2.theta2_dot
    );
    state = temp_state;
    computeAccelerations(clamped_torque, a1, a2);
    k3 = State(temp_state.theta1_dot, a1, temp_state.theta2_dot, a2);
    
    // k4
    temp_state = State(
        curr_state.theta1 + dt * k3.theta1,
        curr_state.theta1_dot + dt * k3.theta1_dot,
        curr_state.theta2 + dt * k3.theta2,
        curr_state.theta2_dot + dt * k3.theta2_dot
    );
    state = temp_state;
    computeAccelerations(clamped_torque, a1, a2);
    k4 = State(temp_state.theta1_dot, a1, temp_state.theta2_dot, a2);
    
    // Update state with RK4 formula
    state.theta1 = curr_state.theta1 + (dt / 6.0) * (k1.theta1 + 2*k2.theta1 + 2*k3.theta1 + k4.theta1);
    state.theta1_dot = curr_state.theta1_dot + (dt / 6.0) * (k1.theta1_dot + 2*k2.theta1_dot + 2*k3.theta1_dot + k4.theta1_dot);
    state.theta2 = curr_state.theta2 + (dt / 6.0) * (k1.theta2 + 2*k2.theta2 + 2*k3.theta2 + k4.theta2);
    state.theta2_dot = curr_state.theta2_dot + (dt / 6.0) * (k1.theta2_dot + 2*k2.theta2_dot + 2*k3.theta2_dot + k4.theta2_dot);
}

void DoublePendulum::computeAccelerations(double torque, double& a1, double& a2)
{
    double theta1 = state.theta1;
    double theta1_dot = state.theta1_dot;
    double theta2 = state.theta2;
    double theta2_dot = state.theta2_dot;
    
    double sin1 = std::sin(theta1);
    double cos1 = std::cos(theta1);
    double sin2 = std::sin(theta2);
    double cos2 = std::cos(theta2);
    double sin12 = std::sin(theta1 - theta2);
    double cos12 = std::cos(theta1 - theta2);
    
    // Lagrangian dynamics for double pendulum with torque on lower arm
    double denom = m1 + m2 * (1.0 - cos12 * cos12);
    
    a1 = (m2 * L2 * theta2_dot * theta2_dot * sin12 * cos12 
         + m2 * g * sin2 * cos12 
         - m2 * L1 * theta1_dot * theta1_dot * sin12
         - (m1 + m2) * g * sin1 
         - b1 * theta1_dot) / (L1 * denom);
    
    a2 = (-m2 * L2 * theta2_dot * theta2_dot * sin12 
         - (m1 + m2) * g * sin1 * cos12 
         + (m1 + m2) * L1 * theta1_dot * theta1_dot * sin12 
         + (m1 + m2) * g * sin2 
         + torque 
         - b2 * theta2_dot) / (L2 * denom);
}

double DoublePendulum::getUpperJointX() const
{
    return L1 * std::sin(state.theta1);
}

double DoublePendulum::getUpperJointY() const
{
    return -L1 * std::cos(state.theta1);
}

double DoublePendulum::getLowerJointX() const
{
    return L1 * std::sin(state.theta1) + L2 * std::sin(state.theta2);
}

double DoublePendulum::getLowerJointY() const
{
    return -L1 * std::cos(state.theta1) - L2 * std::cos(state.theta2);
}
