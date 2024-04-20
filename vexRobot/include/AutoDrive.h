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

    /// @brief  Rotates to the relative angle  ONLY using encoders on wheels (no odometry).
    ///     Good for turning that needs to be fast and might never get to position (ramming). 
    /// @param relative_angle  Double that is the counterclockwise angle in degrees RELATIVE to 
    ///     the current robot heading.
    /// @param velocity  Velocity to turn at (%)
    void turn_relative(double relative_angle, double velocity);

    

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
