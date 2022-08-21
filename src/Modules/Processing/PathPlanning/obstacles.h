#ifndef PROJECT_UNIFICATION_OBSTACLES_H
#define PROJECT_UNIFICATION_OBSTACLES_H

#include <QDebug>
#include <QLine>
#include <vector>
//#include <eigen3/Eigen/Dense>
#include <assert.h>
#include "constants.h"
#include "Modules/Modules.h"

// using namespace Eigen;
using namespace std;

class Obstacles {
 public:
  Obstacles();
  void addObstacle(Point firstPoint, Point secondPoint);
  bool isSegmentInObstacle(Point& p1, Point& p2);
  vector<pair<Point, Point>> obstacles;
};

#endif // PROJECT_UNIFICATION_OBSTACLES_H
