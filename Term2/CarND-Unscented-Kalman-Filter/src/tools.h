#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools {
public:
  // Constructor.
  Tools();

  // Destructor.
  virtual ~Tools();

  // Calculate RMSE.  
  VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

  // Coordinate conversion
  VectorXd Convert_CartesianToPolarCTRV(const VectorXd& x_state);
  VectorXd Convert_CartesianToPolar(const VectorXd& x_state);
  VectorXd Convert_PolarToCartesian(const VectorXd& x_state);
};

#endif /* TOOLS_H_ */