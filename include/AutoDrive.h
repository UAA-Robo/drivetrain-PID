#pragma once
#include "Drive.h"
#include "vex.h"

/// @brief   Contains the methods for the robot to autonomously navigate.
class AutoDrive : public Drive {

public:
    AutoDrive(Hardware* hardware, RobotConfig* robotConfig, Telemetry* telemetry);

    /// @brief Main function that initiates to robot.
    void drive();

    

private:

    RobotConfig *rc;


    /// @brief Moves the drivetrain STRAIGHT until the distance between the current position add
    ///  (goal) position is almost 0.
    /// @param position Pair of doubles: {X, Y}
    void drive_to_position(std::pair<double, double> position);




};
