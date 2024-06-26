#include <cmath>
#include "Drive.h"

Drive::Drive(Hardware *hardware, RobotConfig *robotConfig, Telemetry *telemetry)
{
    hw = hardware;
    rc = robotConfig;
    tm = telemetry;

    //setup_bot();
    // pid = new PID(rc, tm);
}

std::pair<double, double> Drive::calculate_drivetrain_velocity(std::pair<double, double> velocity_percent)
{
    double vertical_velocity_percent = velocity_percent.first / 100;
    double horizontal_velocity_percent = velocity_percent.second / 110; // To reduce sensitivity when turning 
    if (rc-> ROBOT == SCRAT) {
         horizontal_velocity_percent = velocity_percent.second / 110; // To reduce sensitivity when turning
    }

    // Calculate raw left and right motor velocity
    double raw_left_velocity = vertical_velocity_percent + horizontal_velocity_percent;
    double raw_right_velocity = vertical_velocity_percent - horizontal_velocity_percent;

    // Normalize the motor velocity
    double max_raw_velocity = std::max(std::abs(raw_left_velocity), std::abs(raw_right_velocity));
    double normalization_factor = max_raw_velocity > 1.0 ? max_raw_velocity : 1.0;

    double left_velocity_multiplier = raw_left_velocity / normalization_factor;
    double right_velocity_multiplier = raw_right_velocity / normalization_factor;

    // Multiplying by 100 breaks it?? Why?
    double left_velocity = 95 * left_velocity_multiplier;
    double right_velocity = 95 * right_velocity_multiplier;

    return {left_velocity, right_velocity};
}

void Drive::move_drivetrain(std::pair<double, double> velocity_percent)
{
    std::pair<double, double> velocity = calculate_drivetrain_velocity(velocity_percent);

    hw->left_drivetrain_motors.spin(vex::directionType::fwd, velocity.first, 
                                    vex::velocityUnits::pct);
    hw->right_drivetrain_motors.spin(vex::directionType::fwd, velocity.second, 
                                    vex::velocityUnits::pct);
    
    // if (rc->USE_PID_IN_USERDRIVE) pid->correct_drive();
}


