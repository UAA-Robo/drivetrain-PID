#include "PID.h"

PID::PID(Hardware* hardware, float setpoint, float min_control, float max_control, float P, float I, float D) {
    hw = hardware;
    log = new Logger(hw, "PID.csv", {"P", "I", "D", "Control", "Process", "Setpoint"}); 

    this->setpoint = setpoint;
    this->min_control = min_control;
    this->max_control = max_control;

    this->P = P;
    this->I = I;
    this->D = D;
}


double PID::update_control_value(double process_value) {

        float timestamp = hw->brain.timer(vex::timeUnits::msec);
        float change_in_time = timestamp - previous_timestamp;
        previous_timestamp = timestamp;

        process = process_value;

        float error = process - setpoint;
        float derivative = (error - previous_error) / change_in_time;

        integral += error  * change_in_time;

        previous_error = error;

        control = (P * error + I * integral + D * derivative);

        if (control > max_control) control = max_control;
        else if (control < min_control) control = min_control;

        log->add_data({P, I, D, control, process, setpoint}); // TODO: will this take to long?

        return control;
}