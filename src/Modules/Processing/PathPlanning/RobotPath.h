#ifndef PROJECT_UNIFICATION_ROBOTPATH_H
#define PROJECT_UNIFICATION_ROBOTPATH_H

#include "Modules/Modules.h"

using namespace std;

class RobotPath {
 public:
  RobotPath();
  Point lastPoint;
  Point nextPoint;
  std::vector<Point> pathNodes;
  int currentNode;
};

#endif // PROJECT_UNIFICATION_RRTSTAR_H
