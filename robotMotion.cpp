#include <gtsam/geometry/Pose2.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>


using namespace std;
using namespace gtsam;

int main()
{

// Create an empty nonlinear factor graph
NonlinearFactorGraph graph;

// create the prior information of pose x1
Pose2 priorMean(0.0,0.0,0.0);

// create prior noise 
// gaussian diagonal 30cm on robot position, 0.1 radians on robot orientation
noiseModel::Diagonal::shared_ptr priorNoise = 
  noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));

// Add this prior information to the graph as a factor
// takes in a key (1), a mean of type pose, and a noise model for the prior density 
graph.add(PriorFactor<Pose2>(1, priorMean, priorNoise)); // add the prior factor to the graph

// add odometry factors
// For simplicity, we will use the same noise model for each odometry factor
gtsam::Pose2 odometry(2.0, 0.0, 0.0);
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