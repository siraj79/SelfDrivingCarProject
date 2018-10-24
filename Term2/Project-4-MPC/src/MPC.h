#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;




class MPC_Configuration {
   public:
    // TODO: Set the timestep length and duration
    size_t N = 20;    // N is the number of timesteps in the horizon.
    double dt = 0.5;  // dt is how much time elapses between actuations 
    // T = n*dt = 10  // T is the duration over which future predictions are made.
    double latency = 0.1;  //100 millisecond latency

    // [x,y,ψ,v,cte,eψ]
    size_t x_position = 0;
    size_t y_position = 1;
    size_t psi_position = 2;
    size_t v_position= 3;
    size_t cte_position = 4;
    size_t epsi_position = 5;


    // This value assumes the model presented in the classroom is used.
    //
    // It was obtained by measuring the radius formed by running the vehicle in the
    // simulator around in a circle with a constant steering angle and velocity on a
    // flat terrain.
    //
    // Lf was tuned until the the radius formed by the simulating the model
    // presented in the classroom matched the previous radius.
    //
    // This is the length from front to CoG that has a similar radius.
    const double Lf = 2.67;

    // Set cost tunning factors
    int cost_cte_factor = 3000;
    int cost_epsi_factor = 500; // made initial portion etc much less snaky
    int cost_v_factor = 1;
    int cost_current_delta_factor = 1;
    int cost_diff_delta_factor = 200;
    int cost_current_a_factor = 1;
    int cost_diff_a_factor = 1;

    //state and actuator variables are stored in one vector of fixed length based on N
    size_t x_start = 0;
    size_t y_start = N;
    size_t psi_start = 2*N;
    size_t v_start = 3*N;
    size_t cte_start = 4*N;
    size_t epsi_start = 5*N;
    size_t delta_start = 6*N;
    size_t a_start = 7*N - 1;


    // Reference cross-track error and orientation error = 0
    double ref_cte = 0;
    double ref_epsi = 0;
    double ref_v = 40;


    // State: [x,y,ψ,v,cte,eψ]
    // Actuators: [δ,a]
    size_t n_vars = N*6 + (2*(N-1));
    size_t n_constraints = N*6;

    MPC_Configuration(size_t N);
};



class MPC {
 public:
  MPC();
  
  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs,MPC_Configuration mpc_conf);
};
#endif /* MPC_H */
