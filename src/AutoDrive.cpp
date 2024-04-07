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

    double distance = tm->get_distance_between_points(tm->get_current_position(), position);
    double voltage = 0; // In volts
    
    // Controls voltage to prevent overshot.
    PID overshoot_PID(  hw, 
                        0,  // Distance setpoint.
                        1.5, // P
                        1.2, // I
                        0.0); // D
    
    Logger overshoot_log(hw, "overshoot.csv", {"distance", "voltage"});

    // Drive until wheel voltage is 0 (PID controls voltage which controls position)
    while (hw->drivetrain.voltage(vex::voltageUnits::volt) > 0)
    {
        voltage = overshoot_PID.update_control_value(distance);
        hw->drivetrain.spin(vex::directionType::fwd, voltage, vex::voltageUnits::volt);
        
        overshoot_log.add_data({distance, voltage}); // TODO: Will this take to long?
        vex::wait(50, vex::timeUnits::msec); // Wait for odometry wheels to update
        distance = tm->get_distance_between_points(tm->get_current_position(), position);
    }

    hw->drivetrain.stop(); // Stop wheels
    vex::wait(50, vex::timeUnits::msec); // Wait for odometry wheels to update
}