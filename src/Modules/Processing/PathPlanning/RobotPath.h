#ifndef PROJECT_UNIFICATION_ROBOTPATH_H
#define PROJECT_UNIFICATION_ROBOTPATH_H

#include "Modules/Modules.h"

using namespace std;

class RobotPath {

 public:
  RobotPath();
  Point getLastPoint();
  void setLastPoint(Point);
  Point getNextPoint();
  void setNextPoint(Point);
  std::vector<Point> getPathNodes();
  void setPathNodes(std::vector<Point>);
  int getCurrentNode();
  void setCurrentNode(int);

 private:
  Point lastPoint;
  Point nextPoint;
  std::vector<Point> pathNodes;
  int currentNode;
};

#endif // PROJECT_UNIFICATION_RRTSTAR_H
