#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->p_error = 0.0;
  this->i_error = 0.0;
  this->d_error = 0.0;

  this->error = 0.0;

  this->Kp = Kp;
  this->Kd = Ki;
  this->Ki = Kd;

  this->timestep = 0;
}

void PID::UpdateError(double cte, int htime) {

  this->d_error = cte - this->p_error;
  this->p_error = cte;
  
  if (this->i_array.size() >= htime) {
    this->i_array.pop_back();
  }
  this->i_array.push_front(cte);

  this->i_error = 0.0;
  for (int j = 0; j < i_array.size(); ++j) {
    this->i_error += i_array[j];
  }
  
  this->timestep += 1;
}

double PID::TotalError() {
  return this->Kp*this->p_error + this->Kd*this->d_error + this->Ki*this->i_error;
}

