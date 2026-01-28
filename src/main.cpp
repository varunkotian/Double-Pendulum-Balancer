#define _USE_MATH_DEFINES
#include "DoublePendulum.h"
#include "MPC_Controller.h"
#include "ImGuiRenderer.h"
#include <iostream>
#include <chrono>
#include <iomanip>
#include <cmath>

int main()
{
    std::cout << "=== MPC Double Pendulum Balancer ===" << std::endl;
    std::cout << "Initializing system..." << std::endl;

    // Create the double pendulum system
    DoublePendulum pendulum;
    
    // Create the MPC controller
    MPC_Controller controller(&pendulum, 200);
    controller.time_step = 0.01;
    controller.Q_angle = 1000.0;
    controller.Q_angular_vel = 10.0;
    controller.R = 0.1;
    
    // Create ImGui renderer
    ImGuiRenderer renderer(1280, 1280);
    if (!renderer.initialize())
    {
        std::cerr << "Failed to initialize renderer" << std::endl;
        return -1;
    }
    
    std::cout << "Renderer initialized successfully" << std::endl;
    
    // Simulation parameters
    double dt = 0.01;  // Time step (10ms)
    double simulation_time = 0.0;
    double max_simulation_time = 60.0;
    double control_update_interval = 0.01;  // Update input every 0.01 seconds
    double time_since_last_control_update = 0.0;
    double last_torque = 0.0;  // Store the last computed torque
    
    // Pendulum starts at bottom position (initialized in constructor)
    
    std::cout << "System initialized. Starting control loop..." << std::endl;
    std::cout << "Goal: Balance the double pendulum in upright position" << std::endl;
    std::cout << "Close the window to exit." << std::endl;

    auto start_time = std::chrono::high_resolution_clock::now();
    int frame_count = 0;

    while (renderer.isRunning() && simulation_time <= max_simulation_time)
    {
        // Get current state
        State state = pendulum.getState();
        
        // Update control every 0.1 seconds
        double torque = last_torque;
        double cost = 0.0;
        if (time_since_last_control_update >= control_update_interval)
        {
            // Compute control using MPC
            torque = controller.computeControl(state);
            last_torque = torque;
            
            // Compute cost for display
            cost = controller.simulateAndComputeCost(state, torque);
            
            time_since_last_control_update = 0.0;
        }
        
        // Apply control and update dynamics
        pendulum.update(dt, torque);
        
        // Render
        renderer.render(&pendulum, torque, cost, simulation_time);
        
        simulation_time += dt;
        time_since_last_control_update += dt;
        frame_count++;

        // Print status every 100 frames
        if (frame_count % 100 == 0)
        {
            std::cout << "Time: " << std::fixed << std::setprecision(2) << simulation_time 
                      << "s | theta1: " << std::setprecision(4) << state.theta1 
                      << " | theta2: " << state.theta2 << " | Torque: " << torque << " N·m" << std::endl;
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    std::cout << "\n" << std::string(80, '=') << "\n";
    std::cout << "Simulation Complete\n";
    std::cout << std::string(80, '=') << "\n";
    std::cout << "Simulation time: " << simulation_time << " seconds\n";
    std::cout << "Wall-clock time: " << duration.count() << " milliseconds\n";
    std::cout << "Total frames: " << frame_count << "\n";
    if (duration.count() > 0)
        std::cout << "Average FPS: " << (frame_count * 1000.0 / duration.count()) << "\n";
    
    // Print final state
    State final_state = pendulum.getState();
    std::cout << "\nFinal State:\n";
    std::cout << "  Upper Arm Angle (theta1): " << final_state.theta1 << " rad (" 
              << (final_state.theta1 * 180.0 / M_PI) << "deg)\n";
    std::cout << "  Lower Arm Angle (theta2): " << final_state.theta2 << " rad (" 
              << (final_state.theta2 * 180.0 / M_PI) << "deg)\n";

    
    return 0;
}
