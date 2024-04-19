#pragma once
#include "Drive.h"
#include "vex.h"
#include "PID.h"

/// @brief   Contains the methods for the robot to autonomously navigate.
class AutoDrive : public Drive {

public:
    AutoDrive(Hardware* hardware, RobotConfig* robotConfig, Telemetry* telemetry);

    /// @brief Main function that tells robot to drive
    void drive();

    // Perform gradient descent
    void tune_PID_with_gradient_descent();
    
    void random_PID();

    void hand_tune_PID();

    

private:

    RobotConfig *rc;


    /// @brief Moves the drivetrain STRAIGHT until the distance between the current position add
    ///  (goal) position is almost 0.
    /// @param position Pair of doubles: {X, Y}
    /// @param P 
    /// @param I 
    /// @param D 
    /// @param timeout  Timeout in ms
    /// @return Time taken to run (ms)
    double drive_to_position_PID(std::pair<double, double> position,  double P, double I, double D, double timeout);




};
