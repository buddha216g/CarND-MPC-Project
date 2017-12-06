#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

#define REF_CTE 0
#define REF_EPSI 0
#define REF_V 75

// Set weights parameters for the cost function
#define W_CTE 5000   //increase to remove cte
#define W_EPSI 4000  //increase to remove oscillations
#define W_DV 1000    // increase to remove sharp turns at high speeds
#define W_DELTA 5
#define W_A 5
#define W_DDELTA 200 // increase to remove sharp turns
#define W_DA 1000  //increase to remove sudden accelration or decelration

// Set lower and upper limits for variables.
#define DED25RAD 0.436332 // 25 deg in rad, used as delta bound
#define MAXTHR 1.0 // Maximum a value
#define BOUND 1.0e19 // Bound value for other variables


using namespace std;

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
