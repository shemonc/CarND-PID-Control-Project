#ifndef PID_H
#define PID_H

#include<time.h>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
   * Corrected Steering value
   */
  double steer;

  double throttle;

  /*
   * total cross track error
   */
  double total_cte;

  /*
   * Best error
   */
  double best_err;

  /*
   * Previous error
   */
  double cte_prev;

  double prev_time;
  double now_time;

typedef enum controller_ {
    P_CONTROLLER = 0,
    PD_CONTROLLER,
    PI_CONTROLLER,
    TOTAL_CONTROLLER
} controller_t;

/*
 * Each controller has their own state maintained during twiddling
 * 
 */
typedef enum update_state_type_ {
    INVALID_STAGE = 0,

    /*
     * a controller coefficient Kp or Kd or Ki will increment in this stage
     * by their corresponding delta param i.e. p_error or i_error or d_error
     * accordingly
     */
    STAGE1,

    /*
     * if current cross track error(cte) is less than the best error,
     * update the best error and increment the corresponding error
     * paramater, the deltal param
     *
     * if current cte is not better than best decrement the
     * corresponding error parameter twice and mark this controller
     * state to STAGE3 for next evaluation.
     */
    STAGE2,

    /*
     * if current cte is less than the best error, update the best error and
     * increment the corresponding error paramater 
     * else
     * increment the corresponding error paramater to an extent which is less
     * than the previous increment, mark the current state of this controller
     * to STAGE1
     */
    STAGE3
} update_state_type_t;

/*
 * Associate a controller with their current state in twiddle steps.
 */
typedef struct controller_state_ {
    int update_stage;
} controller_state_t;

controller_state_t ctrl_state[TOTAL_CONTROLLER];

/*
 * counter to track the currently evaluating controller
 */
int     current_controller;

/*
 * Number of idle steps from speed 0 to a certain point when
 * twiddle need to start and update the controller parameter
 */
long    steps;

/*
 * current speed of the car
 */
double  speed;

    /*
     * Constructor
     */
    PID();

    /*
     * Destructor.
     */
    virtual ~PID();

    /*
     * Initialize PID.
     */
    void Init(double Kp, double Ki, double Kd);

    /*
     * Update the PID error variables given cross track error.
     */
    void UpdateError(double cte);

    /*
     * Calculate the total PID error.
     */
    double TotalError();

    /*
     * twiddle algorithm
     */
    void Twiddle(double cte);

    /*
     * update error per controller
     */
    void update_controller_error(double err, double &ctrl,
                                double &ctrl_err, int &state);
    /*
     * Apply throttle based on current cross track error
     */
    void ApplyAdaptiveThrottle (double cte);

    /*
     * Apply brake based on current cross track error
     */
    void ApplyAdaptiveBrake (double cte); 
};

#endif /* PID_H */
