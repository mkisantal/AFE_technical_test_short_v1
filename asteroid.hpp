#ifndef ASTEROID_HPP
#define ASTEROID_HPP

#include <Eigen/Dense>
#include "filter.hpp"

// Asteroid class, for tracking. Essentially a wrapper around a KalmanFilter.
class Asteroid{
public:

  int id=-1;

  // Initializing from range+bearing observations and sun position
  // Note: tracking is done in a non-accelerating inertial frame,
  //       origin at Spaceship start position, x is aligned with
  //       sun position.
  Asteroid(SensorData observations, int init_sun_position){

    id = observations.id;

    // getting initial relative position
    std::array<double, 2>  init_pos = observations.getXY(init_sun_position);

    InitKalmanFilter(init_pos[0], init_pos[1]);
    std::cout << "Asteroid " << id <<" initialized. 🌔" << std::endl;

  }

  // Setting up the Kalman filter with matrices accoridng to Asteroid properties.
  void InitKalmanFilter(double init_x, double init_y, double dt=0.2){

    // x0: initial state vector
    Eigen::MatrixXd x0{4,1};
    x0(0, 0) = init_x;
    x0(1, 0) = init_y;
    x0(2, 0) = 0.0;
    x0(3, 0) = 0.0;

    // A: state transition matrix
    Eigen::MatrixXd A(4,4);
    A.setIdentity();
    A(0, 2) = dt;
    A(1, 3) = dt;

    // B: control input matrix, no control.
    Eigen::MatrixXd B(4,1);
    B.setZero();

    // H: observation matrix
    Eigen::MatrixXd H(2,4);
    H.setZero();
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;

    // Q: process noise cov. matrix
    Eigen::MatrixXd Q(4,4);
    Q.setIdentity();
    Q *= 0.1;

    // R: measurement noise cov. matrix
    Eigen::MatrixXd R(2,2);
    R.setIdentity();
    R *= 20.0;

    // Initial estimate error covariance
    Eigen::MatrixXd P0(4,4);
    P0.setIdentity();
    P0.topLeftCorner(2, 2) *= 10.0;
    P0.bottomRightCorner(2, 2) *= 10.0;

    kf = KalmanFilter{A, B, H, Q, R, P0, x0};
  }


  // Updating state with observed position coordinates
  double Propagate(double obs_x, double obs_y, bool update=false){
    Eigen::MatrixXd u(1, 1);
    u.setZero();

    Eigen::MatrixXd z(2, 1);
    z(0, 0) = obs_x;
    z(1, 0) = obs_y;

    KalmanFilter::IntermediateResults res = kf.Update(u, z, update);  // x_posteriori, P_posteriori, residual 

    return res.residual;
  }

  // Updating state according to sensor observations, ship state and sun position
  double Propagate(SensorData observations, int sun_position, const Eigen::MatrixXd& spacecraft_state,
                   bool update=false){
    
    // Get relative position, then shift according to spaceship state
    // since tracking has to be done in inertial frame
    std::array<double, 2> xy = observations.getXY(sun_position);
    xy[0] += spacecraft_state(0, 0);
    xy[1] += spacecraft_state(1, 0);
    
    return Propagate(xy[0], xy[1], update);
  }

  // Filling vector with asteroid relative state
  void WritePosvel(std::array<double, 4>& posvel, Eigen::MatrixXd ss_state){

    Eigen::MatrixXd x = kf.GetState();

    double hdg = -ss_state(4, 0);

    Eigen::MatrixXd relative = x;

    // we were tracking in inertial frame, now need to convert to ship relative
    posvel[0] = cos(hdg) * relative(0, 0) - sin(hdg) * relative(1, 0);
    posvel[1] = sin(hdg) * relative(0, 0) + cos(hdg) * relative(1, 0);
    posvel[2] = cos(hdg) * relative(2, 0) - sin(hdg) * relative(3, 0);
    posvel[3] = sin(hdg) * relative(2, 0) + cos(hdg) * relative(3, 0);

    return;
  }

private:
  KalmanFilter kf;
  
};



#endif