#pragma once
#include "vex.h"
#include "Hardware.h"
#include "Logger.h"

/// @brief   PID controller
class PID {

public:
    PID(Hardware* hardware, float setpoint, float min_control, float max_control, float P, float I, float D);


    /// @brief  Approximation of PID to change the control value based on the new value. 
    ///     NOTE: Requires the brain timer to not be messed with in other parts of the code.
    /// @param process_value Value affected by the control value (what want to reach the setpoint).
    /// @return  Control value which affects the process value. 
    double update_control_value(double process_value);


private:
    Hardware* hw;
    Logger* log; 
    double setpoint = 0;  // What your goal value is (i.e. distance from target == 0 )
    double process = 0;  // What value you are trying to affect (i.e. distance from target)
    double control = 0;  // What thing you change to effect that value (i.e. velocity of drivetrain)

    double min_control = 0;
    double max_control = 0;

    double P = 0;
    double I = 0;
    double D = 0; 

    // Variables that affect the PID over time
    double integral = 0.0;
    double previous_error = 0.0;
    double previous_timestamp = 0.0; // Timestamp that control value was updates previously in msec

};
