#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

#if DEBUG
#define MPC_DEBUG(fn, log) std::cout << "MPC : " << fn << " : " << log << "\n";
#else
#define MPC_DEBUG(fn, log);
#endif

// TODO: Set the timestep length and duration

const size_t N = 25;     // Number of timesteps in the horizon
                         // determines the number of variables optimized byt the MPC
                         // major driver of computational cost

const double dt = 0.05;  // the time elapses between actuation
                         // MPC attempts to approximate a continuous reference trajectory 
                         // by means of discrete path between actuation.
                         // large dt means less frequent actuation, harder to accurately approximate a continuous reference trajectory:

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

// Both the reference cross track and orientation errors are 0.
// The reference velocity is set to 40 mph.
const double ref_v = 40;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuations.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
