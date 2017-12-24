#include <iostream>
#include <cmath>
#include <time.h>
#include "PID.h"

using namespace std;

/*
 * Complete the PID class.
 */

PID::PID() {}

PID::~PID() {}

void 
PID::Init (double Kp, double Ki, double Kd)
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    p_error = 0.3;
    i_error = 0.1;
    d_error = 0.3;

    /*
     * Total cross track error
     */
    total_cte = 0.0;

    /*
     * Put best_err to some higher number
     */
    best_err = 100000;

    /*
     *  default steering value
     */
    steer = 0.0;

    /*
     * Initialize all controller state in STAGE1.
     * Twiddle will be used to find the optimal value
     * for each parameter.
     */
    ctrl_state[P_CONTROLLER].update_stage = STAGE1;
    ctrl_state[PD_CONTROLLER].update_stage = STAGE1;
    ctrl_state[PI_CONTROLLER].update_stage = STAGE1;
    
    /*
     * During twiddling this flag current_controller will indicate
     * for which controller it is evaluating the optimal value
     * start with P_CONTROLLER
     */
    current_controller = P_CONTROLLER;
   
    /*
     * idle steps count before twiddle will start evaluating
     * and contribute to P, I, D coefficients.
     */
    steps = 0;

    /*
     * Default throttle value
     */
    throttle = 0.3;

    speed = 0.0;
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

void
PID::Twiddle (double cte)
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

/*
 * Apply throttle based on current cross track error
 * Assumption is that if the steerign is at max 1 or -1
 * throttle should be minimum and vice versa
 */
void
PID::ApplyAdaptiveThrottle (double cte)
{
    if (fabs(cte) < 0.1) {
        throttle = (1.0 - fabs(steer))*1.0;
    } else if (fabs(cte) < 0.2) {
        throttle = (1.0 - fabs(steer))*0.9;
    } else if (fabs(cte) < 0.3) {
        throttle = (1.0 - fabs(steer))*0.8;
    } else if (fabs(cte) < 0.4) {
        throttle = (1.0 - fabs(steer))*0.4;
    } else if (fabs(cte) < 0.5 ) {
        throttle = (1.0 - fabs(steer))*0.3;
    } else if (fabs(cte) < 0.8 ) {
        throttle = 0.25;
    } else {
        throttle = 0.1;
    }
}

/*
 * Apply brake based on current cross track error
 */
void
PID::ApplyAdaptiveBrake (double cte)
{
    if (speed < 12.0 && throttle <= 3.0 && fabs(steer) < 2) {

        /*
         * minimal speed and throttle but steering is more than max,
         * this will lead the car to stalling, cann't break
         */
        cout<< "NO BRAKE !!"<<endl;
        return;
    }
    
    if (fabs(cte) > 6) {
        throttle = -0.4;
    } else if (fabs(cte) > 5) {
        throttle = -0.3;
    } else if (fabs(cte) > 4) {
        throttle = -0.12; 
    } else if (fabs(cte) > 3 ) {
        throttle = -0.1;
    } else if (fabs(cte) > 2) {
        throttle = -0.09;
    } else if (fabs(cte) > 1.5) {
        throttle = -0.08;
    } else if (fabs(cte) > 1) {
        throttle = -0.07;
    } else {
        throttle = 0.1; 
    }
}

void
PID::UpdateError (double cte) 
{
    double diff_cte;
    double steer_current = 0.0;

    steps++;

    /*
     * Let the car run naturally from speed 0 to some extent until
     * it goes out of lane or the CTE becomes higher. Both of the
     * threshold bellow 100 and 1.9 has estimated by observing the
     * car start in different Graphics Quality i.e.
     * Simple verses Fantastic, in Fantastic mode the cte increases
     * quicker than simple mode so need to start the twiddle earlier.
     */
    if (steps < 100 && cte < 1.9) {
        cout << "Steps " << steps <<endl;
        cte_prev = cte;
        return;
    }

    /*
     * only apply twiddle if the cte is higher than some threshold
     * value, set to 1.2 here my manual experiment and observation.
     */
    if (fabs(cte) > 1.2) {
        Twiddle(cte);
    }

    /*
     * default throttle but will be evaluated bellow based on
     * current cte
     */
    throttle = 0.3;
    diff_cte = cte - cte_prev;
    cte_prev = cte;
    total_cte += cte;

    /*
     * Estimated steering based on current PID Coefficients
     */
    steer_current = -Kp*cte - Kd*diff_cte - Ki*total_cte;

    /*
     * if estimated steering is within -1 to 1, update the current
     * steering value with the estimated value and alos apply 
     * adaptive Throttle
     */
    if (fabs(steer_current) <= 1.0 ) {
        steer = steer_current;
        ApplyAdaptiveThrottle(cte);
    } else {
        
        /*
         * estimated steering is not within -1 to 1, use the previous
         * steering value, reset the PIDs to initial state and apply
         * adaptive brake.
         */
        p_error = 0.2;
        i_error = 0.1;
        d_error = 0.3;
        Kp = 0.2;
        Kd = 5.0;
        Ki = 0.002;
        ApplyAdaptiveBrake(cte);
    }
}

double
PID::TotalError ()
{

    return (p_error + d_error + i_error);
}


