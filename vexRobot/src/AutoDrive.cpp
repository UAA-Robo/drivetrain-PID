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

struct Adam
{
    // Internal state
    float m = 0.0f;
    float v = 0.0f;
 
    // Internal state calculated for convenience
    // If you have a bunch of derivatives, you would probably want to store / calculate these once
    // for the entire gradient, instead of each derivative like this is doing.
    float beta1Decayed = 1.0f;
    float beta2Decayed = 1.0f;
 
    float GetAdjustedDerivative(float derivative, float alpha)
    {
        // Adam parameters
        static const float c_beta1 = 0.9f;
        static const float c_beta2 = 0.999f;
        static const float c_epsilon = 1e-8f;
 
        // exponential moving average of first and second moment
        m = c_beta1 * m + (1.0f - c_beta1) * derivative;
        v = c_beta2 * v + (1.0f - c_beta2) * derivative * derivative;
 
        // bias correction
        beta1Decayed *= c_beta1;
        beta2Decayed *= c_beta2;
        float mhat = m / (1.0f - beta1Decayed);
        float vhat = v / (1.0f - beta2Decayed);
 
        // Adam adjusted derivative
        return alpha * mhat / (std::sqrt(vhat) + c_epsilon);
    }
};


void AutoDrive::tune_PID_with_gradient_descent() {
    const int ITERATIONS = 100;
    const float LEARNING_RATE = 0.001;
    const float MIN = 0.00005;
    const float MAX = 10.5;
    const int TIMEOUT = 3000; // ms
    const int distance = 36; // Inches

    float P = 0.3, I = 0.0, D = 0.2;
    float prev_P = 0,  prev_I = 0, prev_D = 0;
    float E = 0; // Error (settling time)
    float prev_E = 0;

    Adam adamP, adamD; // Adam optimizers for P and D

    Logger log(hw, "gradient_descent.csv", {"Distance (in)", "P", "I", "D", "Over/undershoot (in)"});

    for (int i = 0; i < ITERATIONS; i++) {
        clear_and_set_screen(hw, P, D); // Clear the screen and display P and D

        tm->set_position({0,0});
        tm->set_heading(0);

        E = drive_to_position_PID({distance * (i % 2 == 0 ? 1 : -1), 0}, P, I, D, TIMEOUT);

        log.add_data({distance, P, I, D, E});


        // Compute gradients
        float gradP = -(E - prev_E) / (P - prev_P);
        float gradD = -(E - prev_E) / (D - prev_D);

        // Update P and D using Adam
        P = std::clamp(P + adamP.GetAdjustedDerivative(gradP, LEARNING_RATE), MIN, MAX);
        D = std::clamp(D + adamD.GetAdjustedDerivative(gradD, LEARNING_RATE), MIN, MAX);

        // Logging and waiting
        std::cout << "P: " << P << ", D: " << D << std::endl;
        prev_E = E;
        prev_P = P;
        prev_D = D;
        
        vex::wait(1000, vex::timeUnits::msec); // Wait 1 sec
        turn_relative(180, 15); // Turn around
        vex::wait(3000, vex::timeUnits::msec); // Wait 3 sec
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