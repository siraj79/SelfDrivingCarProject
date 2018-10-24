#ifndef PID_H
#define PID_H


#include "Eigen/Dense"
using Eigen::VectorXd;



class PID {

  public:
    //PID controller needs two kinds of data: configuration and state. The configuration contains the adjustable settings that persist over time. 

    // Errors
    double p_error=0.0;
    double i_error=0.0;
    double d_error=0.0;

    // Coefficients
    double Kp;
    double Ki;
    double Kd;


  private:
    //Smoothing
    double prev_total_error = 0;



  public :
    // Constructor
    PID();

    // Destructor.
    virtual ~PID();

    // Initialize PID.
    void Init(double Kp, double Ki, double Kd);

    // Update the PID error variables given cross track error.
    void UpdateError(double cte);

    // Calculate the total PID error.
    double TotalError();
    double TotalError(bool applyLimit, double lowLimit, double highLimit, bool applySmoothing, double lagcut);
};



class twiddle {

  public:

    VectorXd Params = VectorXd(5);
    VectorXd Prev_Params = VectorXd(5);
    VectorXd dParams = VectorXd(5);

    int number_of_error_added = 0;
    double error;
    double best_err;  

    // Constructor  
    twiddle();

    // Destructor.
    virtual ~twiddle();

    // Initialize PID.
    void Init(VectorXd Params);

    VectorXd Optimize_Parameter();
    void Add_to_Total_Error(double cte);  

  private:
    int increase = 1;
    int current_paramToOptimize_num = 0;    
};


#endif /* PID_H */
