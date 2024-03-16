#include "AutoDrive.h"

AutoDrive::AutoDrive(Hardware *hardware, RobotConfig *robotConfig, Telemetry *telemetry) 
    : Drive(hardware, robotConfig, telemetry) {
    rc = robotConfig;
}

void AutoDrive::drive() {}

void AutoDrive::drive_to_position(std::pair<double, double> position) {}