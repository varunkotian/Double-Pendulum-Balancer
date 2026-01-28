#ifndef DOUBLE_PENDULUM_H
#define DOUBLE_PENDULUM_H

// Simple state vector (no Eigen dependency)
struct State
{
    double theta1;       // Upper arm angle
    double theta1_dot;   // Upper arm angular velocity
    double theta2;       // Lower arm angle
    double theta2_dot;   // Lower arm angular velocity
    
    State() : theta1(0), theta1_dot(0), theta2(0), theta2_dot(0) {}
    State(double t1, double t1d, double t2, double t2d) 
        : theta1(t1), theta1_dot(t1d), theta2(t2), theta2_dot(t2d) {}
};

class DoublePendulum
{
public:
    DoublePendulum();
    
    // State management
    void update(double dt, double torque);
    void setState(const State& state);
    State getState() const;
    
    // Get positions of pendulum joints
    double getUpperJointX() const;
    double getUpperJointY() const;
    double getLowerJointX() const;
    double getLowerJointY() const;
    
    // Physical parameters
    double L1 = 0.5;  // Length of upper arm (meters)
    double L2 = 0.5;  // Length of lower arm (meters)
    double m1 = 1.0;  // Mass of upper arm (kg)
    double m2 = 1.0;  // Mass of lower arm (kg)
    double g = 9.81;  // Gravity (m/s^2)
    double b1 = 0.1;  // Friction coefficient for upper joint
    double b2 = 0.1;  // Friction coefficient for lower joint

private:
    State state;  // Current system state
    
    // Dynamics equations
    void computeAccelerations(double torque, double& a1, double& a2);
};

#endif // DOUBLE_PENDULUM_H
