---
marp: true
theme: gaia
class: invert
size: 16:9
math: katex
# header: 'Header content'
footer: 'RCVL - Toronto Metropolitan University'
title: 'GTSAM Factor Graphs'
paginate: true
---

# Factor Graphs and GTSAM
- Before we begin, let's build GTSAM (since it takes some time)

**Install GTSAM**
Add PPA
```
sudo add-apt-repository ppa:borglab/gtsam-develop
sudo apt update
```
Install:
`sudo apt install libgtsam-dev libgtsam-unstable`

---

## Factor Graphs
- A factor graph is a bipartite graphical model representing the factorization of a function
- Can be used to model complex inference (estimation) problems
- Variables,  $X_n$, can represent unknown desired variables
- Factors, $f_n$,  connect these variables and define their relationships
- For example, a joint probability distribution can be represented as
$$
p(V)=\prod_{j}^{N_{f}} f_{j}\left(V_{j}\right)
$$


---
## Factor Graphs

![bg h:50%](./factor_graphs.drawio.svg)

---
## GTSAM 
- GTSAM is a sensor fusion library based on factor graphs
- Very large library, has support for SLAM, Kalman filtering, Lie algebra etc.
- Documentation is lacking, but lots of examples provided
- Main modules: `Base`, `Geometry`, `Navigation`, `SLAM`, `Nonlinear`
- Their use is best explained through examples

---
## Examples: 
We will demonstrate the use of GTSAM through a few examples
1. Simple Rotation
2. Modelling Robot Motion
3. Robot Localization 
4. SLAM



---
### 1. Simple Rotation
This example is a simple example of optimizing a single variable with a single prior factor 
- Variables: $X_1$, with represents a 2D rotation 
- Factors: $f_0$

We will create a goal angle of $30\degree$, with an initial guess of $20\degree$

---
### 1. Simple Rotation Cont'd
The factor graph would look like:
![bg w:60%](./factors_3.drawio.svg) 
<br/><br/> 
<br/> 
You can open `GTSAM-tutorial/src/SimpleRotation.cpp` to follow along

---
<style scoped>section { font-size: 30px; }</style>
### 1. Simple Rotation
Header includes:
- `#include <gtsam/geometry/Rot2.h>`: 2D rotation is variable of interest
- `#include <gtsam/inference/Symbol.h>`: Simple header for symbols (X1 X2 etc)
- `#include <gtsam/nonlinear/NonlinearFactorGraph.h>:` Our factors are nonlinear
- `#include <gtsam/nonlinear/Values.h>`: GTSAM requires we use an initial guess for each variable, which we must hold in a Values container.
- `#include <gtsam/nonlinear/LevenbergMarquardtOptimier.h>`: MAP solver used (there are others)

---
### 1. Simple Rotation Cont'd
Create the unary factor (measurement data from sensor)
To create a factor we need:
  1. A key or set of keys to label the variables
  2. A measurement value
  3. A measurement model
```c++
Rot2 prior = Rot2::fromAngle(30 * degree);  // create measurement value
prior.print("goal angle");
auto model = noiseModel::Isotropic::Sigma(1, 1 * degree); // create measurement model
Symbol key('x', 1); // create key
```

---
### 1. Simple Rotation Cont'd
Now we can create the graph container and add the factor to it
```c++
NonlinearFactorGraph graph;
graph.addPrior(key, prior, model);
graph.print("full graph");
```

---
### 1. Simple Rotation Cont'd 
Create an initial estimate before optimizing
```c++
Values initial;
initial.insert(key, Rot2::fromAngle(20*degree));
initial.print("initial estimate");
```
Now optimize
```c++
Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
result.print("final result");
```

---
### 1. Simple Rotation Cont'd
The output is as follows
<style scoped>section { font-size: 30px; }</style>

```bash
goal angle: 0.523599
full graphsize: 1

Factor 0: PriorFactor on x1
  prior mean: : 0.523599
isotropic dim=1 sigma=0.0174533

initial estimateValues with 1 values:
Value x1: (gtsam::Rot2) : 0.349066

final resultValues with 1 values:
Value x1: (gtsam::Rot2) : 0.523599

```

---
#### 2. Modelling Robot Motion
- Variables: $x_1, x_2, x_3$ representing robot pose
- Factors
  - $f_0(x_1)$ , a unary factor on pose $x_1$ encoding our prior knowledge of the state
  - $f_1(x_1,x_2:o_1)$, a binary factor relating $x_1$ and $x_2$, and odom data $o_1$
  - $f_2(x_2,x_3:o_2)$, similar to $f_1$

A visual representation is in the next slide

---
#### Modelling Robot Motion

![bg w:95%](./factor_graph_2.drawio.svg)
<\br/>
<\br/>
<\br/>
Open `robotMotion.cpp` to follow along

---
#### Importing libraries
`#include <gtsam/geometry/Pose2.h>`: We will use Pose2 variables (x, y, theta) to represent the robot positions
`#include <gtsam/slam/BetweenFactor.h>`: 

