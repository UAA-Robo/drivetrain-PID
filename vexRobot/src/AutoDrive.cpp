#include "AutoDrive.h"

AutoDrive::AutoDrive(Hardware *hardware, RobotConfig *robotConfig, Telemetry *telemetry) 
    : Drive(hardware, robotConfig, telemetry) {
    rc = robotConfig;
}

void AutoDrive::drive() {
    tune_PID_with_gradient_descent();
    //random_PID();
    //hand_tune_PID();
}

void AutoDrive::hand_tune_PID() {
    float P = 0.24; // Oscillates at 12V at 0.48
    float D = 1;

    float I = 0.0;

    tm->set_position({0,0});
    tm->set_heading(0);

    drive_to_position_PID({36, 0}, P, I, D, 5000);

}

void AutoDrive::tune_PID_with_gradient_descent() {

    const int ITERATIONS = 100;
    const float LEARNING_RATE = 0.001;


    const float MIN = 0.05;
    const float MAX = 1.5;

    float P = 0.3;
    float I = 0.0;
    float D = 0.2;
    float E = 0; // Error (settling time)

    float prev_P = 0;
    //float prev_I = I;
    float prev_D = 0;
    float prev_E = 0;

    float TIMEOUT = 3000; // 10,000ms = 10s timeout
   // float E_AT_TIMEOUT = 2000; // Large number for E if it times out (basically 20 seconds)
    int direction = 1;
    const int distance = 36; // Inches
    
    Logger log(hw, "gradient_descent.csv", {"Distance (in)", "P", "I", "D","Over/undershoot (in)"}); 

    // Only tuning PD for now
    // Drive forward and backward
    for (int i = 0; i < ITERATIONS; i++) {
        hw->controller.Screen.clearScreen();
        hw->controller.Screen.setCursor(1,1);
        hw->controller.Screen.print("P: %.4lf", P);
        hw->controller.Screen.setCursor(2,1);
        hw->controller.Screen.print("D: %.4lf\n", D);

        tm->set_position({0,0});
        tm->set_heading(0);

        //i % 2 == 0 ? direction = 1 : direction = -1; // Forward on even is, backward on negative i
        E = drive_to_position_PID({distance, 0}, P, I, D, TIMEOUT); // * Direction
        //E = fabs(E); // Distance to goal
        //if (E >= TIMEOUT || E <= 500) E = E_AT_TIMEOUT; // If E greater than timeout or didn't move, set large error

        log.add_data({distance, P, I, D, E});
        hw->controller.Screen.setCursor(3,1);
        hw->controller.Screen.print("E: %.4lf\n", E);

    
        float P_diff = P - prev_P;
        prev_P = P;
        if (P_diff == 0) P = P - LEARNING_RATE; // Avoid divide by 0 error
        else P = P - (E - prev_E) / P_diff * LEARNING_RATE;

        if (P > MAX) P = MAX;
        else if (P < MIN) P = MIN;

        float D_diff = D - prev_D;
        prev_D = D;
        if (D_diff == 0) D = D - LEARNING_RATE; // Avoid divide by 0 error
        else D = D - (E - prev_E) / D_diff * LEARNING_RATE;

        if (D > MAX) D = MAX;
        else if (D < MIN) D = MIN;

        std::cout <<"P: " << P <<  ", D: " << D << std::endl;

        prev_E = E;
        
        vex::wait(1000, vex::timeUnits::msec); // Wait 3 sec
        turn_relative(180, 15); // Turn around to prep for next run
        vex::wait(3000, vex::timeUnits::msec); // Wait 3 sec
        
    }


}

void AutoDrive::random_PID() {

    float I = 0.0;


    float TIMEOUT = 5000; // 5,000ms = 5s timeout
    int direction = 1;
    const int distance = 36; // Inches
    float settling_time = 0;

    const float MIN = 0.0;
    const float MAX = 1.5;
    const float STEP = 0.1;
    int i = 0;

    Logger log(hw, "random_PID.csv", {"Distance (in)", "P", "I", "D","Settling_Time (ms)"}); 


    for (double P=MIN; P <= MAX; P+= STEP) {

        for (double D=MIN; D <=MAX; D += STEP) {
            tm->set_position({0,0});
            tm->set_heading(0);

            i % 2 == 0 ? direction = 1 : direction = -1; // Forward on even is, backward on negative i
            i++;

            settling_time = drive_to_position_PID({distance * direction, 0}, P, I, D, TIMEOUT);
            log.add_data({distance, P, I, D, settling_time});

            hw->controller.Screen.clearScreen();
            hw->controller.Screen.setCursor(1,1);
            hw->controller.Screen.print("P: %.4lf", P);
            hw->controller.Screen.setCursor(2,1);
            hw->controller.Screen.print("D: %.4lf\n", D);

            vex::wait(1000, vex::timeUnits::msec); // Wait 1 sec
        }
    }
}


double AutoDrive::drive_to_position_PID(std::pair<double, double> position, double P, double I, double D, double timeout) {

    double distance;
    double voltage; // In volts

    double init_timestamp = hw->brain.timer(vex::timeUnits::msec);

    // Controls voltage to prevent overshot.
    PID overshoot_PID(  hw, 
                        0,  // Distance setpoint
                        -12, // Max output voltage
                        12, // Min output voltage
                        P, // P
                        I, // I
                        D); // D

    // Drive until wheel voltage is with +/- 0.1V of 0 (PID controls voltage which controls position)
    do {
        distance = tm->get_signed_distance_to_position(position);
        voltage = overshoot_PID.update_control_value(distance);
        hw->drivetrain.spin(vex::directionType::fwd, voltage, vex::voltageUnits::volt);
        
        vex::wait(50, vex::timeUnits::msec); // Wait for odometry wheels to update
    }
    // While not stopped and < timeout
    while (fabs(hw->drivetrain.voltage(vex::voltageUnits::volt) - 0) >= 0.1 
            && hw->brain.timer(vex::timeUnits::msec) -  init_timestamp <= timeout);

    hw->drivetrain.stop(); // Stop wheels
    vex::wait(50, vex::timeUnits::msec); // Wait for odometry wheels to update
    
    //return hw->brain.timer(vex::timeUnits::msec) -  init_timestamp;
    return tm->get_signed_distance_to_position(position);
}


void AutoDrive::turn_relative(double relative_angle, double velocity) {
    // Determines whether to rotate left or right based on the  shortest distance
    if (360 - fabs(relative_angle) < relative_angle) relative_angle = relative_angle - 360;
    //if (relative_angle > 180) 
    double revolutions = relative_angle  * (rc->DRIVETRAIN_WIDTH) * M_PI 
        / (360 * rc->WHEEL_CIRCUMFERENCE) * rc->DRIVETRAIN_GEAR_RATIO_MULTIPLIER;

    hw->left_drivetrain_motors.resetPosition();
    hw->right_drivetrain_motors.resetPosition();

    hw->left_drivetrain_motors.spinFor(-revolutions, vex::rotationUnits::rev, velocity, 
        vex::velocityUnits::pct, false);
    hw->right_drivetrain_motors.spinFor(revolutions, vex::rotationUnits::rev, velocity, 
        vex::velocityUnits::pct);

    // Blocks other tasks from starting
    while (fabs(hw->left_drivetrain_motors.velocity(vex::velocityUnits::pct)) > 0 
        || fabs(hw->right_drivetrain_motors.velocity(vex::velocityUnits::pct)) > 0); 

}