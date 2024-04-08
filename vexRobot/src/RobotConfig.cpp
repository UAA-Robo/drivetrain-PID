#include "RobotConfig.h"

RobotConfig::RobotConfig(Hardware* hardware)
{
    hw = hardware;
    ODOMETRY_CIRCUMFERENCE  = 2.75 * M_PI; // Odometry wheel circumference
    //WHEELCIRC = 3.25 * M_PI; // Drive train wheel circumference in inches --> CHECK: might be 3 inches!

    auto_drive_velocity_percent = 20;
    auto_rotate_velocity_percent = 20;

    std::cout << "Robot is SCRAT\n";
    ROBOT = SCRAT;

    INTAKE_WAIT = 500;

    /*
    ----SET DIMENSIONS (in) --------------------------------------------------------------
    */
        // Distance between left and right odometry wheels over 2. It was measured and then tuned 
        // to get accurate turning results
    ODOMETRY_LEFT_RIGHT_RADIUS =  6.45 / 2;

    // Distance between horizontal odometry wheel and horizontal center line
    ODOMETRY_BACK_RADIUS = 4.125; 
    ODOMETRY_BACK_OFFSET = 0;

    WHEEL_CIRCUMFERENCE = 3.25 * M_PI;

    // Distance (in inch) between left and right side of the drivetrain 
    // (measured from the center of the wheels)
    DRIVETRAIN_WIDTH = 11.75;

    // Max width
    ACTUAL_WIDTH = 14.50; 

    MAX_CATAPULT_ANGLE = 64;
    
    ARCING_FORWARD_BACKWARD_MULTIPLIER = 0.4;
    ARCING_LEFT_RIGHT_MULTIPLIER = 0.3;
    IN_PLACE_LEFT_RIGHT_MULTIPLIER = 0.5;

    DRIVETRAIN_GEAR_RATIO_MULTIPLIER = 0.33; // 36:1


    DRIVETRAIN_RADIUS = DRIVETRAIN_WIDTH * sqrt(2) / 2.0;
    ACTUAL_RADIUS = ACTUAL_WIDTH * sqrt(2) / 2.0;

    hw = hardware;
}