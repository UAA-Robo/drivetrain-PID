#include "UserDrive.h"
#include "iostream"

UserDrive::UserDrive(Hardware *hardware, RobotConfig *robotConfig, Telemetry *telemetry) 
    : Drive(hardware, robotConfig, telemetry) {}

void UserDrive::drive()
{

    while(true) {
        drivetrain_controls();
        vex::wait(20, vex::msec);  // Wait necessary to give time to other threads
    }
}

void UserDrive::drivetrain_controls() {
    const int DEADZONE = 2;
    float forward_backward = hw->controller.Axis3.position(vex::percentUnits::pct);
    float left_right = hw->controller.Axis1.position(vex::percentUnits::pct);

    if (std::abs(forward_backward) < DEADZONE) {
        forward_backward = 0;
    }

    if (std::abs(left_right) < DEADZONE) {
        left_right = 0;
    }

    // Keep responsive arcing
    if (forward_backward != 0 && left_right != 0 && !INTAKE_EXPANDED && !INTAKE_HELD) {
        left_right /= left_right_joystick_multiplier;
        left_right *= rc->ARCING_LEFT_RIGHT_MULTIPLIER;
        forward_backward *= rc->ARCING_FORWARD_BACKWARD_MULTIPLIER;
    }
    

    move_drivetrain(
        {
            forward_backward * fwd_bwd_joystick_multiplier,
            left_right * left_right_joystick_multiplier
        }
    );
}
