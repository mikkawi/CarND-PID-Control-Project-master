#ifndef PID_H
#define PID_H

#include <vector> 

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
  * helping parameters
  */
  int it; // keep count of iterations
  double error; // keep track of average error after each itiration
  double best_error;// keep track of best error
  bool twiddle; // switch between parameter assessing mode and self driving mode

  std::vector<double> dp; // vector of twiddle movements
  std::vector<double> p;  // vector of parameters

  double tol; // accepted tolerance for twiddle 

  int parameter_index; // since we cannot loop on the parameters similar to class, we loop using %3

  bool p_increase, p_decrease; // used to control twiddle paramter change
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
};

#endif /* PID_H */
