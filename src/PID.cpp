#include "PID.h"

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  //Initialization of PID coefficients and errors

    this->Kp = Kp_;
    this->Ki = Ki_;
    this->Kd = Kd_;

    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;
}

void PID::UpdateError(double cte) {
   // Updation of PID errors based on cte.
    
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;

}

double PID::TotalError() {
   //Calculation of total error
  
  double total_error = Kp*p_error + Ki*i_error + Kd*d_error;
  return total_error;  
}
