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

**Clone and build GTSAM**
```
mkdir ~/projects/
git clone https://github.com/borglab/gtsam
cd gtsam
git checkout 4.0.3
mkdir build && cd build
cmake ..
sudo make install
cd ../..
```
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
#### Modelling Robot Motion
- Variables: $x_1, x_2, x_3$ representing robot pose
- Factors
  - $f_0(x_1)$ , a unary factor on pose $x_1$ encoding our prior knowledge of the state
  - $f_1(x_1,x_2:o_1)$, a binary factor relating $x_1$ and $x_2$, and odom data $o_1$
  - $f_2(x_2,x_3:o_2)$, similar to $f_1$

A visual representation is in the next slide

---
#### Modelling Robot Motion

![bg w:95%](./factor_graph_2.drawio.svg)

#### Creating the factor graph
Open `robotMotion.cpp`
