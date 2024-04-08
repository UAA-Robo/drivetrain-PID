#include "AutoDrive.h"

AutoDrive::AutoDrive(Hardware *hardware, RobotConfig *robotConfig, Telemetry *telemetry) 
    : Drive(hardware, robotConfig, telemetry) {
    rc = robotConfig;
}

void AutoDrive::drive() {
    tm->set_position({0,0});
    tm->set_heading(0);

    // Drive 3 ft
    drive_to_position({36, 0});
}


void AutoDrive::drive_to_position(std::pair<double, double> position) {

    double distance;
    double voltage; // In volts
    
    // Controls voltage to prevent overshot.
    PID overshoot_PID(  hw, 
                        0,  // Distance setpoint
                        -12, // Max output voltage
                        12, // Min output voltage
                        0.5, // P
                        0.0, // I
                        0.0); // D

    // Drive until wheel voltage is with +/- 0.1V of 0 (PID controls voltage which controls position)
    do {
        distance = tm->get_signed_distance_to_position(position);
        voltage = overshoot_PID.update_control_value(distance);
        hw->drivetrain.spin(vex::directionType::fwd, voltage, vex::voltageUnits::volt);
        
        vex::wait(50, vex::timeUnits::msec); // Wait for odometry wheels to update
    }
    while (fabs(hw->drivetrain.voltage(vex::voltageUnits::volt) - 0) >= 0.1);

    hw->drivetrain.stop(); // Stop wheels
    vex::wait(50, vex::timeUnits::msec); // Wait for odometry wheels to update
}