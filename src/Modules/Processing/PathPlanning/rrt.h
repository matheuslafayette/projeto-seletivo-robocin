#ifndef RRT_H
#define RRT_H

#include "./Modules/Modules.h"
#include "obstacles.h"
#include <stdlib.h>
#include <vector>
#include <math.h>
#include <iostream>

using namespace std;
// using namespace Eigen;

struct Node {
  vector<Node*> children;
  Node* parent;
  Point position;
  float orientation;
  double cost;
};

class RRT {
 public:
  RRT();
  void initialize();
  Node* getRandomNode();
  Node* nearest(Point point);
  int distance(Point& p, Point& q);
  Point newConfig(Node* q, Node* qNearest);
  void add(Node* qNearest, Node* qNew);
  bool reached();
  void setStepSize(int step);
  void setMaxIterations(int iter);
  void deleteNodes(Node* root);
  vector<Point> runRRT();
  void near(Point point, float radius, vector<Node*>& out_nodes);
  void setInitPos(Point startPos);
  void setEndPos(Point endPos);
  double Cost(Node* q);
  double PathCost(Node* qFrom, Node* qTo);
  Obstacles* obstacles;
  vector<Node*> nodes;
  vector<Node*> path;
  Node *root, *lastNode;
  Point startPos, endPos;
  int max_iter;
  int step_size;
};

#endif // RRT_H
