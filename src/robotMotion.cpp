// We will use Pose2 variables (x, y, theta) to represent the robot positions
#include <gtsam/geometry/Pose2.h>

// Here we will use Between factors for the relative motion described by odometry measurements.
// Also, we will initialize the robot at the origin using a Prior factor.
#include <gtsam/slam/BetweenFactor.h>

// since our factors are nonlinear, we need a nonlinear factor graph
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// map solver
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

// we can also calculate marginals covariance
#include <gtsam/nonlinear/Marginals.h>

// we need to store initial guesses in a Values container
#include <gtsam/nonlinear/Values.h>


using namespace std;
using namespace gtsam;

int main()
{

// Create an empty nonlinear factor graph
NonlinearFactorGraph graph;

// remember, for a factor we need three things:
  // 1. A key or set of keys to label the variables
  // 2. A measurement value
  // 3. A measurement model

// create the prior value of pose 1, setting it to origin
// A prior factor consists of a mean and a noise model (covariance matrix)
Pose2 priorMean(0.0,0.0,0.0);

// create prior noise model  
// gaussian diagonal 30cm on robot position, 0.1 radians on robot orientation
noiseModel::Diagonal::shared_ptr priorNoise = 
  noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));

// we will use a key of int 1 instead of a symbol like before
int key_1 = 1;

// Add this prior information to the graph as a factor
// takes in a key (1), a mean of type pose, and a noise model for the prior density 
graph.add(PriorFactor<Pose2>(key_1, priorMean, priorNoise)); // add the prior factor to the graph

// now we can make factors 2 and 3

// add odometry factors
// For simplicity, we will use the same noise model for each odometry factor

// create odometry pose
gtsam::Pose2 odometry(2.0, 0.0, 0.0);

// create odometry noise model
noiseModel::Diagonal::shared_ptr odometryNoise = 
  noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));

// add a factor between keys (poses) 1 and 2
graph.add(BetweenFactor<Pose2>(1, 2, odometry, odometryNoise));

// add a factor between keys (poses) 2 and 3
graph.add(BetweenFactor<Pose2>(2, 3, odometry, odometryNoise));
graph.print("\nFactor Graph:\n"); // print

// we can also optimize using different methods
 // Create the data structure to hold the initialEstimate estimate to the solution
  // For illustrative purposes, these have been deliberately set to incorrect values
  Values initial;
  initial.insert(1, Pose2(0.5, 0.0, 0.2));
  initial.insert(2, Pose2(2.3, 0.1, -0.2));
  initial.insert(3, Pose2(4.1, 0.1, 0.1));
  initial.print("\nInitial Estimate:\n"); // print

   // optimize using Levenberg-Marquardt optimization
  Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
  result.print("Final Result:\n");


// Full Posterior Inference
  // Calculate and print marginal covariances for all variables
  cout.precision(2);
  Marginals marginals(graph, result);
  cout << "x1 covariance:\n" << marginals.marginalCovariance(1) << endl;
  cout << "x2 covariance:\n" << marginals.marginalCovariance(2) << endl;
  cout << "x3 covariance:\n" << marginals.marginalCovariance(3) << endl;

}