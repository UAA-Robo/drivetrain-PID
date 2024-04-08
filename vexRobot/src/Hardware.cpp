#include "Hardware.h"

Hardware::Hardware()
{
    // Motor breaking

    // Drivetrain
    front_left_drivetrain_motor.setBrake(vex::brakeType::coast);
    middle_left_drivetrain_motor.setBrake(vex::brakeType::coast);
    back_left_drivetrain_motor.setBrake(vex::brakeType::coast);

    front_right_drivetrain_motor.setBrake(vex::brakeType::coast);
    middle_left_drivetrain_motor.setBrake(vex::brakeType::coast);
    back_right_drivetrain_motor.setBrake(vex::brakeType::coast);

    
    left_drivetrain_motors.setStopping(vex::brakeType::coast);
    right_drivetrain_motors.setStopping(vex::brakeType::coast);

    drivetrain.setStopping(vex::brakeType::coast);

    // Set timeout
    left_drivetrain_motors.setTimeout(2, vex::timeUnits::sec);
    right_drivetrain_motors.setTimeout(2, vex::timeUnits::sec);
    drivetrain.setTimeout(2, vex::timeUnits::sec);

    // Odometry
    left_odometry.resetPosition();
    right_odometry.resetPosition();
    back_odometry.resetPosition();


}
