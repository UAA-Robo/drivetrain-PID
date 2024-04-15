#include "AutoDrive.h"

AutoDrive::AutoDrive(Hardware *hardware, RobotConfig *robotConfig, Telemetry *telemetry) 
    : Drive(hardware, robotConfig, telemetry) {
    rc = robotConfig;
}

void AutoDrive::drive() {
    tune_PID_with_gradient_descent();
}

void AutoDrive::tune_PID_with_gradient_descent() {

    const int ITERATIONS = 100;
    const float LEARNING_RATE = 0.01;

    float P = 0;
    float I = 0;
    float D = 0;
    float E = 0; // Error (settling time)

    float prev_P = P;
    //float prev_I = I;
    float prev_D = D;
    float prev_E = E;

    float TIMEOUT = 10000; // 10,000ms = 10s timeout
    float E_AT_TIMEOUT = 1000; // Large number for E if it times out
    int direction = 1;
    const int distance = 36; // Inches
    
    Logger log(hw, "gradient_descent.csv", {"Distance (in)", "P", "I", "D","Settling_Time (ms)"}); 

    // Only tuning PD for now
    // Drive forward and backward
    for (int i = 0; i < ITERATIONS; i++) {
        tm->set_position({0,0});
        tm->set_heading(0);

        i % 2 ? direction = 1 : direction = -1; // Forward on even is, backward on negative i
        E = drive_to_position_PID({distance * direction, 0}, P, I, D, TIMEOUT);
        if (E >= TIMEOUT) E = E_AT_TIMEOUT;

        log.add_data({distance, P, I, D, E});
        hw->controller.Screen.clearScreen();
        hw->controller.Screen.setCursor(0,0);
        hw->controller.Screen.print("P: %.4lf\n", P);
        hw->controller.Screen.print("D: %.4lf\n", D);
        hw->controller.Screen.print("E: %.4lf\n", E);

    
        P = P - (E - prev_E) / (P - prev_P) * LEARNING_RATE;
        //I = P - (E - prev_E) / (I - prev_I) * LEARNING_RATE;
        P = D - (E - prev_E) / (D - prev_D) * LEARNING_RATE;

        prev_E = E;
        prev_P = P;
        //prev_I = I;
        prev_D = D;

        vex::wait(5000, vex::timeUnits::msec); // Wait 5 msec
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
    return hw->brain.timer(vex::timeUnits::msec) -  init_timestamp <= timeout;
}