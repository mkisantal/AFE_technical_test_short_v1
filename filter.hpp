#ifndef FILTER_HPP
#define FILTER_HPP


#include <Eigen/Dense>
#include <iostream>
#include <stdexcept>
#include <deque>
#include <numeric>


// A simple moving average filter
template<typename T>
class AverageFilter{
public:
  AverageFilter(size_t N): N(N) {}

  T GetState(){
    while (history.size() > N){
      history.pop_front();
    }
    T sum = std::accumulate(history.begin(), history.end(), 0.0);
    return sum/(T)N;
  }

  void Update(T new_val){
    history.push_back(new_val);
  }

private:
  size_t N;  // length of history kept
  std::deque<T> history;  // stored values

};


// Class for Kalman Filtering
class KalmanFilter {
public:

  KalmanFilter(const Eigen::MatrixXd& A,
               const Eigen::MatrixXd& B,
               const Eigen::MatrixXd& H,
               const Eigen::MatrixXd& Q,
               const Eigen::MatrixXd& R,
               const Eigen::MatrixXd& P0,
               const Eigen::MatrixXd& x0) : A(A), B(B), H(H), Q(Q), R(R), P(P0), x(x0), I(A) {
    num_states = A.rows();
    num_ctrls = B.cols();
    num_obs = H.rows();
    I.setIdentity(); // shape from A, set to identity matrix
    initialized = true;

  }

  KalmanFilter() { initialized = false; };

  // Struct used to return a posteriori results and residual
  struct IntermediateResults {
    Eigen::MatrixXd x_posteriori;
    Eigen::MatrixXd P_posteriori;
    double residual;
    IntermediateResults(Eigen::MatrixXd x, Eigen::MatrixXd P, Eigen::MatrixXd res):
                                                              x_posteriori(x), P_posteriori(P) {
      residual = res.norm();
    }
  };

  // Normal Kalman Filter update calculations
  IntermediateResults Update(Eigen::MatrixXd u, Eigen::MatrixXd z, bool apply=true){

    // Predict
    Eigen::MatrixXd x_priori = A * x + B * u;
    Eigen::MatrixXd P_priori = A * P * A.transpose() + Q;

    // Update
    Eigen::MatrixXd K = P_priori * H.transpose() * (H * P_priori * H.transpose() + R).inverse(); 
    Eigen::MatrixXd residual = (z - H * x_priori);
    
    Eigen::MatrixXd x_posteriori = x_priori + K * residual;
    Eigen::MatrixXd P_posteriori = (I - K * H) * P_priori;
    
    if (apply){
      x = x_posteriori;
      P = P_posteriori;
    }

    return IntermediateResults(x_posteriori, P_posteriori, residual);
  }




  Eigen::MatrixXd GetState(){
    return x;
  }


private:

  // Number of states, observations and control inputs
  int num_states, num_obs, num_ctrls;
  bool initialized = false;

  Eigen::MatrixXd A, B, H;
  Eigen::MatrixXd Q, R;
  Eigen::MatrixXd P, x;
  Eigen::MatrixXd I;


};


#endif