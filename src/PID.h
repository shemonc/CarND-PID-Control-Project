#ifndef PID_H
#define PID_H

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

typedef enum controller_ {
    P_CONTROLLER = 0,
    PD_CONTROLLER,
    PI_CONTROLLER,
    TOTAL_CONTROLLER
} controller_t;

typedef enum update_state_type_ {
    INVALID_STAGE = 0,
    STAGE1,
    STAGE2,
    STAGE3
} update_state_type_t;

typedef struct controller_state_ {
    int update_stage;
} controller_state_t;

controller_state_t ctrl_state[TOTAL_CONTROLLER];

int  current_controller;
long steps;
long twiddle_settle;

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
     * twiddle
     */
    void Twiddle(double cte);

    /*
     * update error per controller
     */
    void update_controller_error(double err, double &ctrl,
                                double &ctrl_err, int &state);
};

#endif /* PID_H */
