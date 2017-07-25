#ifndef PID_H
#define PID_H

#include <list>
#include <cmath>

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

private:
  double previous_cte;
  double total_cte;
  double max_steering_angle;
  std::list<double> cte_vec;
  double AttenuateSteer(double steer_value);
  int i_history_len;
};

#endif /* PID_H */
