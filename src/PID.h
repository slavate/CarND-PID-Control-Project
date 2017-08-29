#ifndef PID_H
#define PID_H

#include <deque>

class PID {
public:
  /*
  * Errors
  */
  double p_error; // also prev_cte
  double i_error; // also int_cte
  std::deque<double> i_array;
  double d_error; // also diff_cte
  double error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Helpers
  */
  int timestep;

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
  void Init(double p, double i, double d);

  /*
  * Update the PID error variables given cross track error and predefined history time.
  */
  void UpdateError(double cte, int htime);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */
