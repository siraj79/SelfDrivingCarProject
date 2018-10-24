#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;




MPC_Configuration::MPC_Configuration(size_t N) {
   this->N = N;
}




class FG_eval {
 public:

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector; 
  // configuration  
  MPC_Configuration mpc_conf;
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;


  FG_eval(Eigen::VectorXd coeffs,size_t N) : mpc_conf(N) { 
    this->coeffs = coeffs; 
  }

  // `fg` a vector of the cost constraints
  // `vars` is a vector of variable values (state & actuators)
  void operator()(ADvector& fg, const ADvector& vars) {

    #pragma Step 1 : Assign Cost
      // Initialise cost to zero
      fg[0] = 0;
      for(size_t i=0 ; i<mpc_conf.N ; i++)
      {
        //Cost of 'distance from reference state'
        fg[0] += mpc_conf.cost_cte_factor   *  pow(vars[mpc_conf.cte_start + i]  - mpc_conf.ref_cte,  2);  // Cost of CTE : error between the center of the road and the vehicle's position
        fg[0] += mpc_conf.cost_epsi_factor  *  pow(vars[mpc_conf.epsi_start + i] - mpc_conf.ref_epsi, 2);  // Cost of deviation from orientation angle 
        fg[0] += mpc_conf.cost_v_factor     *  pow(vars[mpc_conf.v_start + i]    - mpc_conf.ref_v,    2);  // Cost of deviation from Reference velocity

        //Cost of 'use of actuators'
        if(i< mpc_conf.N - (size_t)1)
        {
          fg[0] += mpc_conf.cost_current_delta_factor  *  pow(vars[mpc_conf.delta_start + i], 2);  // Steering Angle : Actuator = Steering Wheel
          fg[0] += mpc_conf.cost_current_a_factor      *  pow(vars[mpc_conf.a_start + i], 2);      // Acceleration   : Actuator = Throttel & Break Padel 
        } 

        //Cost of latency (delay as the command propagates through the system)
        if(i< mpc_conf.N - (size_t)2)
        {
          fg[0] += mpc_conf.cost_diff_delta_factor  *  pow(vars[mpc_conf.delta_start + i + 1] - vars[mpc_conf.delta_start + i], 2);
          fg[0] += mpc_conf.cost_diff_a_factor      *  pow(vars[mpc_conf.a_start + i + 1]     - vars[mpc_conf.a_start + i], 2);
        }
      }
    #pragma endregion   

    #pragma Step 2 : Set initial state 
      fg[1 + mpc_conf.x_start]    = vars[mpc_conf.x_start];
      fg[1 + mpc_conf.y_start]    = vars[mpc_conf.y_start];
      fg[1 + mpc_conf.psi_start]  = vars[mpc_conf.psi_start];
      fg[1 + mpc_conf.v_start]    = vars[mpc_conf.v_start];
      fg[1 + mpc_conf.cte_start]  = vars[mpc_conf.cte_start];
      fg[1 + mpc_conf.epsi_start] = vars[mpc_conf.epsi_start];
    #pragma endregion 

    #pragma Step 3 : Set predicted states
      // N - 1 because we're only predicting (N-1) times
      for (size_t i = 0; i < mpc_conf.N - 1; i++) {
        // The state at time t+1 .
        AD<double> x1    = vars[mpc_conf.x_start + i + 1];
        AD<double> y1    = vars[mpc_conf.y_start + i + 1];
        AD<double> psi1  = vars[mpc_conf.psi_start + i + 1];
        AD<double> v1    = vars[mpc_conf.v_start + i + 1];
        AD<double> cte1  = vars[mpc_conf.cte_start + i + 1];
        AD<double> epsi1 = vars[mpc_conf.epsi_start + i + 1];
        
        // The state at time t.
        AD<double> x0    = vars[mpc_conf.x_start + i];
        AD<double> y0    = vars[mpc_conf.y_start + i];
        AD<double> psi0  = vars[mpc_conf.psi_start + i];
        AD<double> v0    = vars[mpc_conf.v_start + i];
        AD<double> cte0  = vars[mpc_conf.cte_start + i];
        AD<double> epsi0 = vars[mpc_conf.epsi_start + i];
        
        // Only consider the actuation at time t.
        AD<double> delta0 = vars[mpc_conf.delta_start + i];
        AD<double> a0     = vars[mpc_conf.a_start + i];
             
        AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * pow(x0,2) + coeffs[3] * pow(x0,3);
        AD<double> psides0 = CppAD::atan(coeffs[1] + (2 * coeffs[2] * x0) + (3 * coeffs[3]* pow(x0,2) ));
        
        // Fill in fg with differences between actual and predicted states
        // equations for the model:
        fg[2 + mpc_conf.x_start + i]    = x1 - (x0 + v0 * CppAD::cos(psi0) * mpc_conf.dt);
        fg[2 + mpc_conf.y_start + i]    = y1 - (y0 + v0 * CppAD::sin(psi0) * mpc_conf.dt);
        fg[2 + mpc_conf.psi_start + i]  = psi1 - (psi0 - v0 * delta0 / mpc_conf.Lf * mpc_conf.dt);
        fg[2 + mpc_conf.v_start + i]    = v1 - (v0 + a0 * mpc_conf.dt);
        fg[2 + mpc_conf.cte_start + i]  = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * mpc_conf.dt));
        fg[2 + mpc_conf.epsi_start + i] = epsi1 - ((psi0 - psides0) - v0 * delta0 / mpc_conf.Lf * mpc_conf.dt);
      }
    #pragma endregion
  }  
};




//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs,MPC_Configuration mpc_conf) {
  bool ok = true;
  //size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;



  // TODO: Set the number of model variables (includes both states and inputs).
  size_t n_vars = mpc_conf.n_vars;
  // TODO: Set the number of constraints
  size_t n_constraints = mpc_conf.n_constraints;



  #pragma Set Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    Dvector vars(n_vars);
    for (size_t i = 0; i < n_vars; i++) {
      vars[i] = 0.0;
    }
    vars[mpc_conf.x_start] = state[mpc_conf.x_position];
    vars[mpc_conf.y_start] = state[mpc_conf.y_position];
    vars[mpc_conf.psi_start] = state[mpc_conf.psi_position];
    vars[mpc_conf.v_start] = state[mpc_conf.v_position];
    vars[mpc_conf.cte_start] = state[mpc_conf.cte_position];
    vars[mpc_conf.epsi_start] = state[mpc_conf.epsi_position];
  #pragma endregion


  #pragma Set Lower and upper limits for variables.
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);
    // Set all Non-Actuator to no Limits
      for (size_t i = 0; i < mpc_conf.delta_start; i++) {
      vars_upperbound[i] = INT_MAX;
      vars_lowerbound[i] = -INT_MAX;
    }  
    
    // Steering angle (deltas)
    for (size_t i = mpc_conf.delta_start; i < mpc_conf.a_start; i++)
    {
      vars_upperbound[i] = M_PI/8; // max values allowed in simulator
      vars_lowerbound[i] = -M_PI/8;
    }
    
    // Acceleration
    for (size_t i = mpc_conf.a_start; i < n_vars; i++)
    {
      vars_upperbound[i] = 1.0;
      vars_lowerbound[i] = -1.0;
    }
  #pragma endregion


  #pragma Set Lower and upper limits for the constraints
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (size_t i = 0; i < n_constraints; i++) {
      constraints_lowerbound[i] = 0;
      constraints_upperbound[i] = 0;
    }

      // Set init state lower and upper limits 
    constraints_lowerbound[mpc_conf.x_start] = state[mpc_conf.x_position];
    constraints_lowerbound[mpc_conf.y_start] = state[mpc_conf.y_position];
    constraints_lowerbound[mpc_conf.psi_start] = state[mpc_conf.psi_position];
    constraints_lowerbound[mpc_conf.v_start] = state[mpc_conf.v_position];
    constraints_lowerbound[mpc_conf.cte_start] = state[mpc_conf.cte_position];
    constraints_lowerbound[mpc_conf.epsi_start] = state[mpc_conf.epsi_position];

    constraints_upperbound[mpc_conf.x_start] = state[mpc_conf.x_position];
    constraints_upperbound[mpc_conf.y_start] = state[mpc_conf.y_position];
    constraints_upperbound[mpc_conf.psi_start] = state[mpc_conf.psi_position];
    constraints_upperbound[mpc_conf.v_start] = state[mpc_conf.v_position];
    constraints_upperbound[mpc_conf.cte_start] = state[mpc_conf.cte_position];
    constraints_upperbound[mpc_conf.epsi_start] = state[mpc_conf.epsi_position];
  #pragma endregion




  // object that computes objective and constraints
  FG_eval fg_eval(coeffs,mpc_conf.N);





  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;




  // TODO: Return the first actuator values. The variables can be accessed with
  vector<double> result;  
  result.push_back(solution.x[mpc_conf.delta_start]);
  result.push_back(solution.x[mpc_conf.a_start]);
  
  for (size_t i = 0; i < mpc_conf.N-1; i++)
  {
    result.push_back(solution.x[mpc_conf.x_start + i + 1]);
    result.push_back(solution.x[mpc_conf.y_start + i + 1]);
  }
  return result;
}
