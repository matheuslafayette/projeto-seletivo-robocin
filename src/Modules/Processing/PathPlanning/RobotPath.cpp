#include "./RobotPath.h"

RobotPath::RobotPath() {

  lastPoint = Point(3000, -3000);
  currentNode = 0;
}

Point RobotPath::getLastPoint() {

  return this->lastPoint;
}

Point RobotPath::getNextPoint() {

  return this->nextPoint;
}

std::vector<Point> RobotPath::getPathNodes() {

  return this->pathNodes;
}

int RobotPath::getCurrentNode() {

  return currentNode;
}

void RobotPath::setLastPoint(Point lastPoint) {

  this->lastPoint = lastPoint;
}

void RobotPath::setNextPoint(Point nextPoint) {

  this->nextPoint = nextPoint;
}

void RobotPath::setPathNodes(std::vector<Point> pathNodes) {

  this->pathNodes = pathNodes;
}

void RobotPath::setCurrentNode(int currentNode) {

  this->currentNode = currentNode;
}