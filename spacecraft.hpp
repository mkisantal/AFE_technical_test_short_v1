#ifndef SPACECRAFT_HPP
#define SPACECRAFT_HPP

#include <Eigen/Dense>
#include "filter.hpp"

// Class for state propagation and updates
class Spacecraft{
public:

  // Initialize from sun position (degrees) and timestep (sec)
  Spacecraft(int init_sun_position, double dt=0.2, double thrust=5.0): dt(dt), thrust(thrust) {
    
    A.setIdentity(); //A: State transition matrix
    A(0, 2) = dt;
    A(1, 3) = dt;

    x.setZero(); //x: State vector
    x(4, 0) = - (double)init_sun_position * M_PI / 180.0;

    // Initializing moving average filters for heading.
    // Both sin and cos tracked to avoid periodicity problems.
    sin_theta_filter.Update(sin(x(4, 0)));
    cos_theta_filter.Update(cos(x(4, 0)));

    std::cout << "\n\nSpacecraft initialization 🚀\n---------------" << std::endl;

  }

  // Propagating according to new observed sun position and whether thrust is applied
  Eigen::MatrixXd Propagate(int sun_position, bool accelerate=false){

    // State evolution
    Eigen::MatrixXd x_next = A * x;

    // Nonlinear input
    Eigen::MatrixXd Bu{5,1};
    Bu(0, 0) = accelerate ? (0.5 * thrust * dt * dt * cos(x(4, 0))) : 0.0;
    Bu(1, 0) = accelerate ? (0.5 * thrust * dt * dt * sin(x(4, 0))) : 0.0;
    Bu(2, 0) = accelerate ? (thrust * dt * cos(x(4, 0))) : 0.0;
    Bu(3, 0) = accelerate ? (thrust * dt * sin(x(4, 0))) : 0.0;
    //Instead of setting Bu(4, 0), heading is directly updated, after filtering.
    x_next += Bu;

    double observed_heading = - (double)sun_position * M_PI / 180.0;
    sin_theta_filter.Update(sin(observed_heading));
    cos_theta_filter.Update(cos(observed_heading));

    // heading angle reconstructed from filtered sin and cos
    x_next(4, 0) = atan2(sin_theta_filter.GetState(), cos_theta_filter.GetState());  

    x = x_next;

    return x_next;

  };

private:

  const size_t N = 10;  // num of values stored for filtering
  AverageFilter<double> sin_theta_filter{N};
  AverageFilter<double> cos_theta_filter{N};

  Eigen::MatrixXd A{5,5};  // state transition matrix
  Eigen::MatrixXd x{5,1};  // state vector
   
  double dt;     // sec
  double thrust; // m/s^2

};


#endif