#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

// Set weights parameters for the cost function
#define WEIGHT_CTE 3000
#define WEIGHT_EPSI 3000
#define WEIGHT_V 1
#define WEIGHT_DELTA 10000
#define WEIGHT_A 500
#define WEIGHT_DDELTA 10
#define WEIGHT_DA 10


class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
  vector<double> mpc_x;
  vector<double> mpc_y;
};

#endif /* MPC_H */
