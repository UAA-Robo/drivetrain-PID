#pragma once
#include "vex.h"

enum robot_name {
    SCRAT = 0,
    SCRATETTE = 1
};

/// @brief    Stores the electronics (controller, sensors, motors, etc.) used by the other classes.
class Hardware {

public:
    Hardware();
    vex::brain brain;
    vex::controller controller = vex::controller(vex::controllerType::primary);

    // Entire drivetrain for moving forward/backward evenly
    vex::motor_group drivetrain = vex::motor_group(
        front_left_drivetrain_motor, middle_left_drivetrain_motor, back_left_drivetrain_motor,
        front_right_drivetrain_motor, middle_right_drivetrain_motor, back_right_drivetrain_motor);


    // Left motors
    vex::motor_group left_drivetrain_motors = vex::motor_group(
        front_left_drivetrain_motor, middle_left_drivetrain_motor, back_left_drivetrain_motor);

    // Right motors
    vex::motor_group right_drivetrain_motors = vex::motor_group(
        front_right_drivetrain_motor, middle_right_drivetrain_motor, back_right_drivetrain_motor);

    // NOTE: Scrat is 36:1 so DRIVETRAIN_GEAR_RATIO_MULTIPLIER declared 0.5 in hardware
    // Left drivetrain side
    vex::motor front_left_drivetrain_motor = vex::motor(vex::PORT11, vex::ratio18_1, false);
    vex::motor middle_left_drivetrain_motor = vex::motor(vex::PORT12, vex::ratio18_1, false);
    vex::motor back_left_drivetrain_motor = vex::motor(vex::PORT13, vex::ratio18_1, false);
    // Right drivetrain side
    vex::motor front_right_drivetrain_motor = vex::motor(vex::PORT17, vex::ratio18_1, true);
    vex::motor middle_right_drivetrain_motor = vex::motor(vex::PORT19, vex::ratio18_1, true);
    vex::motor back_right_drivetrain_motor = vex::motor(vex::PORT18, vex::ratio18_1, true);

    // Odometry Wheels
    vex::rotation left_odometry =  vex::rotation(vex::PORT6, true);
    vex::rotation right_odometry =  vex::rotation(vex::PORT7);
    vex::rotation back_odometry =  vex::rotation(vex::PORT8);

};