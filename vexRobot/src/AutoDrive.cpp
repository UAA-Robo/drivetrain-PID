#include "AutoDrive.h"

AutoDrive::AutoDrive(Hardware *hardware, RobotConfig *robotConfig, Telemetry *telemetry) 
    : Drive(hardware, robotConfig, telemetry) {
    rc = robotConfig;
}

void AutoDrive::drive() {
    //tune_PID_with_gradient_descent();
    random_PID();
}

void AutoDrive::tune_PID_with_gradient_descent() {

    const int ITERATIONS = 100;
    const float LEARNING_RATE = 0.01;


    const float MIN = 0.5;
    const float MAX = 1.5;

    float P = 0.5;
    float I = 0.0;
    float D = 0.05;
    float E = 0; // Error (settling time)

    float prev_P = 0;
    //float prev_I = I;
    float prev_D = 0;
    float prev_E = 0;

    float TIMEOUT = 10000; // 10,000ms = 10s timeout
    float E_AT_TIMEOUT = 20000; // Large number for E if it times out (basically 20 seconds)
    int direction = 1;
    const int distance = 36; // Inches
    
    Logger log(hw, "gradient_descent.csv", {"Distance (in)", "P", "I", "D","Settling_Time (ms)"}); 

    // Only tuning PD for now
    // Drive forward and backward
    for (int i = 0; i < ITERATIONS; i++) {
        tm->set_position({0,0});
        tm->set_heading(0);
        
        hw->controller.Screen.clearScreen();
        hw->controller.Screen.setCursor(1,1);
        hw->controller.Screen.print("P: %.4lf", P);
        hw->controller.Screen.setCursor(2,1);
        hw->controller.Screen.print("D: %.4lf\n", D);

        i % 2 == 0 ? direction = 1 : direction = -1; // Forward on even is, backward on negative i
        E = drive_to_position_PID({distance * direction, 0}, P, I, D, TIMEOUT);
        if (E >= TIMEOUT || E <= 500) E = E_AT_TIMEOUT; // If E greater than timeout or didn't move, set large error

        log.add_data({distance, P, I, D, E});
        hw->controller.Screen.setCursor(3,1);
        hw->controller.Screen.print("E: %.4lf\n", E);

    
        float P_diff = P - prev_P;
        if (P_diff == 0) P_diff = 0.000000000001; // Avoid divide by 0 error

        P = P - (E - prev_E) / P_diff * LEARNING_RATE;
        if (P > MAX) P = MAX;
        else if (P < MIN) P = MIN;

        // float I_diff = I - prev_I;
        //if (I_diff == 0)) I_diff = 0.000000000001; // Avoid divide by 0 error
        // I = P - (E - prev_E) / I_diff * LEARNING_RATE;
        // if (I > MAX) I = MAX;
        // else if (I < MIN) I = MIN;

        float D_diff = D - prev_D;
        if (D_diff == 0) D_diff = 0.000000000001; // Avoid divide by 0 error
        D = D - (E - prev_E) / D_diff * LEARNING_RATE;
        if (D > MAX) D = MAX;
        else if (D < MIN) D = MIN;

        std::cout <<"P: " << P <<  ", D: " << D << std::endl;

        prev_E = E;
        prev_P = P;
        //prev_I = I;
        prev_D = D;

        vex::wait(5000, vex::timeUnits::msec); // Wait 5 sec
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
                        -6, // Max output voltage
                        6, // Min output voltage
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
    
    return hw->brain.timer(vex::timeUnits::msec) -  init_timestamp;
}