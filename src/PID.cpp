#include "PID.h"
#include <limits>
#include <numeric>
using namespace std;
#include <cmath>
#include <iostream>

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {

	PID::Kp = Kp;
    PID::Ki = Ki;
    PID::Kd = Kd;
    p_error = 0.0;
    d_error = 0.0;
    i_error = 0.0;
    it = 1;
    error = 0;
    best_error = std::numeric_limits<double>::max(); // maximum possible error
    twiddle = false;

    dp = {0.1*Kp, 0.1*Ki, 0.1*Kd};
    p  = {Kp, Ki, Kd};
    tol = 0.0001;

    parameter_index = 2;

    p_increase = p_decrease = false;
}

void PID::UpdateError(double cte) {

    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;

    error = PID::TotalError();
    //double now_at = std::accumulate(dp.begin(),dp.end(),0);
    double now_at = dp[0]+dp[1]+dp[2];
    cout << "now at " << now_at << endl;
    if(twiddle && now_at > tol && it>=100){
    	
    	cout << "iteration: " << it << endl;
    	cout <<"twiddling: with tolerance: " << tol << " now at: " << now_at << endl;
    	cout << "error: " << error << " best_error: " << best_error << endl;

    	if (error < best_error){
    		cout<<"****************"<<endl;
    		if (it !=100)
                dp[parameter_index] *= 1.1;
    		best_error = error;
    		parameter_index = (parameter_index + 1) % 3;  // loop on parameters in list from 0 to 2
    		p_increase = p_decrease = false;
    	}

    	if (!p_increase && !p_decrease){
    		p[parameter_index] += dp[parameter_index];
    		p_increase = true;

    	}
    	else if (p_increase && !p_decrease){
    		p[parameter_index] -= 2 * dp[parameter_index];
    		p_decrease = true;

    	}
    	else{

    		dp[parameter_index] *= 0.9;
    		p_increase = p_decrease = false;
    		parameter_index = (parameter_index + 1) % 3; 

    	}
    	PID::Kp = p[0];
    	PID::Ki = p[1];
    	PID::Kd = p[2];

    	cout << "kp: " << p[0] << " ki: " << p[1] << " kd: " << p[2] << endl;


    }

    it++;
}

double PID::TotalError() {
	return error += pow(p_error,2)/it;  // total average error
}

