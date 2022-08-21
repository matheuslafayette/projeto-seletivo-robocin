#ifndef PROJECT_UNIFICATION_RRTSTAR_H
#define PROJECT_UNIFICATION_RRTSTAR_H

#include "obstacles.h"
//#include "dubins.h"
#include <stdlib.h>
#include <vector>
#include <math.h>
#include <iostream>
#include <cmath>

using namespace std;
// using namespace Eigen;

struct Node {
  vector<Node*> children;
  Node* parent;
  Point position;
  float orientation;
  double cost;
  // DubinsPath path;
};

class RRTSTAR {
 public:
  RRTSTAR();
  void initialize();
  Node* getRandomNode();
  Node* nearest(Point point);
  void near(Point point, float radius, vector<Node*>& out_nodes);
  double distance(Point& p, Point& q);
  double Cost(Node* q);
  double PathCost(Node* qFrom, Node* qTo);
  Point newConfig(Node* q, Node* qNearest);
  // Point newDubinConfig(Node* q, Node* qNearest, DubinsPath& path);
  void add(Node* qNearest, Node* qNew);
  bool reached();
  void setStepSize(int step);
  void setMaxIterations(int iter);
  void deleteNodes(Node* root);
  void setInitPos(Point startPos);
  void setEndPos(Point endPos);
  vector<Point> runRRTSTAR();
  Obstacles* obstacles;
  vector<Node*> nodes;
  vector<Node*> path;
  Node *root, *lastNode;
  Point startPos, endPos;
  int max_iter;
  int step_size;
  Point target;
};

#endif // PROJECT_UNIFICATION_RRTSTAR_H
