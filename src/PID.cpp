#include <iostream>
#include <cmath>
#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init (double Kp, double Ki, double Kd)
{
    this->Kp = 0;
    this->Ki = 0;
    this->Kd = 0;
    p_error = 0.3;
    i_error = 0.1;
    d_error = 0.3;
    total_cte = 0.0;
    best_err = 100000;
    steer = 0.0;
    twiddle_settle = 0;
    ctrl_state[P_CONTROLLER].update_stage = STAGE1;
    ctrl_state[PD_CONTROLLER].update_stage = STAGE1;
    ctrl_state[PI_CONTROLLER].update_stage = STAGE1;
    current_controller = P_CONTROLLER;
    steps = 0;
}

void PID::update_controller_error (double err, double &ctrl, double &ctrl_err,
                                   int &state)
{
    switch (state) {
    
        case STAGE1:
            ctrl += ctrl_err;
            state = STAGE2;
            break;

        case STAGE2:
            if (err < best_err) {
                best_err = err;               
                ctrl_err *= 1.1;
                current_controller = 
                                (current_controller + 1) % TOTAL_CONTROLLER;
            } else {
                
                ctrl -= 2*ctrl_err;
                state = STAGE3;
            }
            break;

        case STAGE3:
            if (err < best_err) {
                best_err = err;
                ctrl_err *= 1.1;
                current_controller = 
                                (current_controller + 1) % TOTAL_CONTROLLER;
            } else {
                
                ctrl += ctrl_err;
                ctrl_err *= 0.9;
                state = STAGE1;
                current_controller = 
                                (current_controller + 1) % TOTAL_CONTROLLER;
            }

            break;
        default:
            cout << "Error: Invalid state "<< state;
            cout << " for controller type "<< current_controller <<endl;
    }

}

void PID::Twiddle (double cte)
{
    switch (current_controller) {

        case P_CONTROLLER:
            update_controller_error(cte, Kp, p_error,
                                    ctrl_state[P_CONTROLLER].update_stage);
            break;
        case PD_CONTROLLER:
           update_controller_error(cte, Kd, d_error,
                                    ctrl_state[PD_CONTROLLER].update_stage); 
            break;
        case PI_CONTROLLER:
           update_controller_error(cte, Ki, i_error,
                                    ctrl_state[PI_CONTROLLER].update_stage); 
            break;
        default:
            cout << "Error: Invalid controller type ";
            cout << current_controller <<endl;
    }
}
void PID::UpdateError (double cte) 
{
    double diff_cte;
    double steer_current = 0.0;
    steps++;
    if (steps < 100) {
        cout << "Steps " << steps <<endl;
        return;
    }
    if (fabs(cte) > 1) {
        Twiddle(cte);
    }

    diff_cte = cte - cte_prev;
    cte_prev = cte;
    total_cte += cte;
    steer_current = -Kp*cte - Kd*diff_cte - Ki*total_cte;
    if (fabs(steer_current) < 1.0)
        steer = steer_current;
    else {
        cout << ">>>> not setting steering value " << steer_current <<endl;
        cout << "Kp "<< Kp << " Kd "<< Kd << " Ki " << Ki <<endl;
        p_error = 0.3;
        i_error = 0.1;
        d_error = 0.3;
        Kp = 0.2;
        Kd = 3.0;
        Ki = 0.004;
    }
}

double PID::TotalError() {

    return (p_error + d_error + i_error);
}


