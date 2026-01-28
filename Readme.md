# MPC Double Pendulum Balancer

A Model Predictive Control (MPC) based balancer for a double pendulum system with console output.

## Project Structure

```
MPC_basic/
├── CMakeLists.txt              # Build configuration
├── include/
│   ├── DoublePendulum.h         # Double pendulum dynamics
│   ├── MPC_Controller.h         # MPC control algorithm
│   ├── Renderer.h               # Console output handler
│   └── imGuiRendere.h           # GUI to show the current state
│
├── src/
│   ├── main.cpp                 # Application entry point
│   ├── DoublePendulum.cpp       # Physics simulation
│   ├── MPC_Controller.cpp       # MPC implementation
│   └── Renderer.cpp             # Console output implementation
│   └── imGuiRenderer.cpp         # GUI implementation
│
└── external/
    └── imgui/                   # Dear ImGui library (for future use)
```

## Features

- **Double Pendulum Dynamics**: Realistic physics simulation with friction and gravity
- **MPC Control**: Model Predictive Control to balance the pendulum
- **Console Output**: Real-time state display in terminal


## System Details

### State Vector
- `θ1` (theta1): Upper arm angle [rad]
- `θ1_dot`: Upper arm angular velocity [rad/s]
- `θ2` (theta2): Lower arm angle [rad] - **Controlled**
- `θ2_dot`: Lower arm angular velocity [rad/s]

### Control
- Applied torque on the lower arm joint
- Objective: Balance pendulum in upright position (θ1 = 0, θ2 = 0)

### Physical Parameters (Customizable)
- `L1`, `L2`: Arm lengths (default: 0.5 m)
- `m1`, `m2`: Arm masses (default: 1.0 kg)
- `g`: Gravitational acceleration (default: 9.81 m/s²)
- `b1`, `b2`: Joint friction coefficients (default: 0.1)

## MPC Parameters

The MPC controller uses the following cost function:

$$J = \sum_{k=0}^{N} (Q_a (1-cos(\theta_k)) + Q_v \dot{\theta}_k^2 + R u_k^2)$$

Where:
- `Q_a`: Weight for angle errors (default: 1000.0)
- `Q_v`: Weight for angular velocity errors (default: 10.0)
- `R`: Weight for control effort (default: 0.1)
- `u`: Control torque
- `N`: Prediction horizon (default: 200 steps)

## Console Output

The program displays real-time information:
- Simulation time
- Pendulum angles and angular velocities
- Applied control torque
- MPC cost value

## Future Improvements

1. **Faster Optimization**: Implement quadratic programming (QP) solver
2. **iLQG Algorithm**: Implement Iterative Linear Quadratic Gaussian
3. **GUI Visualization**: Re-add Dear ImGui for graphical visualization
4. **Parameter Tuning**: Interactive parameter adjustment
5. **Performance Metrics**: Real-time efficiency calculations

## Mathematical Background

The double pendulum dynamics are derived using the Lagrangian formulation:

- Kinetic energy: $T = \frac{1}{2}I_1\dot{\theta}_1^2 + \frac{1}{2}I_2\dot{\theta}_2^2$
- Potential energy: $V = m_1gh_1 + m_2gh_2$
- Euler-Lagrange equations combined with the applied torque

