#include "rrt.h"

RRT::RRT() {
  obstacles = new Obstacles;
  startPos.setX(START_POS_X);
  startPos.setY(START_POS_Y);
  endPos.setX(END_POS_X);
  endPos.setY(END_POS_Y);
  root = new Node;
  root->parent = NULL;
  root->position = startPos;
  lastNode = root;
  nodes.push_back(root);
  step_size = 3;
  max_iter = 3000;
}

/**
 * @brief Initialize root node of RRT.
 */
void RRT::initialize() {
  root = new Node;
  root->parent = NULL;
  root->position = startPos;
  lastNode = root;
  nodes.push_back(root);
}

/**
 * @brief Generate a random node in the field.
 * @return
 */
Node* RRT::getRandomNode() {
  Node* ret;
  Point point(drand48() * WORLD_WIDTH, drand48() * WORLD_HEIGHT);
  if (point.x() >= 0 && point.x() <= WORLD_WIDTH && point.y() >= 0 && point.y() <= WORLD_HEIGHT) {
    ret = new Node;
    ret->position = point;
    return ret;
  }
  return NULL;
}

/**
 * @brief Helper method to find distance between two positions.
 * @param p
 * @param q
 * @return
 */
int RRT::distance(Point& p, Point& q) {
  Point v = p - q;
  return sqrt(powf(v.x(), 2) + powf(v.y(), 2));
}

/**
 * @brief Get nearest node from a given configuration/position.
 * @param point
 * @return
 */
Node* RRT::nearest(Point point) {
  float minDist = 1e9;
  Node* closest = NULL;
  for (int i = 0; i < (int) nodes.size(); i++) {
    float dist = distance(point, nodes[i]->position);
    if (dist < minDist) {
      minDist = dist;
      closest = nodes[i];
    }
  }
  return closest;
}

/**
 * @brief Get neighborhood nodes of a given configuration/position.
 * @param point
 * @param radius
 * @param out_nodes
 * @return
 */
void RRT::near(Point point, float radius, vector<Node*>& out_nodes) {
  for (int i = 0; i < (int) nodes.size(); i++) {
    double dist = distance(point, nodes[i]->position);
    if (dist < radius) {
      out_nodes.push_back(nodes[i]);
    }
  }
}

/**
 * @brief Find a configuration at a distance step_size from nearest node to random node.
 * @param q
 * @param qNearest
 * @return
 */
Point RRT::newConfig(Node* q, Node* qNearest) {
  Point to = q->position;
  Point from = qNearest->position;
  Point intermediate = to - from;
  intermediate = intermediate / intermediate.norm();
  Point ret = from + step_size * intermediate;
  return ret;
}

/**
 * @brief Add a node to the tree.
 * @param qNearest
 * @param qNew
 */
void RRT::add(Node* qNearest, Node* qNew) {
  qNew->parent = qNearest;
  qNearest->children.push_back(qNew);
  nodes.push_back(qNew);
  lastNode = qNew;
}

/**
 * @brief Check if the last node is close to the end position.
 * @return
 */
bool RRT::reached() {
  if (distance(lastNode->position, endPos) < END_DIST_THRESHOLD)
    return true;
  return false;
}

void RRT::setStepSize(int step) {
  step_size = step;
}

void RRT::setMaxIterations(int iter) {
  max_iter = iter;
}

/**
 * @brief Delete all nodes using DFS technique.
 * @param root
 */
void RRT::deleteNodes(Node* rootDel) {
  for (int i = 0; i < (int) rootDel->children.size(); i++) {
    deleteNodes(rootDel->children[i]);
  }
  delete rootDel;
}

double RRT::Cost(Node* q) {
  return q->cost;
}

double RRT::PathCost(Node* qFrom, Node* qTo) {
  return distance(qTo->position, qFrom->position);
}

void RRT::setInitPos(Point startPos) {

  this->startPos.setX(startPos.x());
  this->startPos.setY(startPos.y());
}

void RRT::setEndPos(Point endPos) {

  this->endPos.setX(endPos.x());
  this->endPos.setY(endPos.y());
}

vector<Point> RRT::runRRT() {

  for (int i = 0; i < max_iter; i++) {
    Node* q = getRandomNode();
    if (q) {
      Node* qNearest = nearest(q->position);
      if (distance(q->position, qNearest->position) > step_size) {
        Point newConfigPosOrient;
        newConfigPosOrient = newConfig(q, qNearest);
        Point newConfigPos(newConfigPosOrient.x(), newConfigPosOrient.y());
        if (!obstacles->isSegmentInObstacle(newConfigPos, qNearest->position)) {
          Node* qNew = new Node;
          qNew->position = newConfigPos;
          qNew->orientation = newConfigPosOrient.angle();
          // qNew->path = path;
          vector<Node*> Qnear;
          near(qNew->position, step_size * RRTSTAR_NEIGHBOR_FACTOR, Qnear);
          // qDebug() << "Found Nearby " << Qnear.size() << "\n";
          Node* qMin = qNearest;
          double cmin = Cost(qNearest) + PathCost(qNearest, qNew);
          for (int j = 0; j < 0 || (unsigned) j < Qnear.size(); j++) {
            Node* qNear = Qnear[j];
            if (!obstacles->isSegmentInObstacle(qNear->position, qNew->position) &&
                (Cost(qNear) + PathCost(qNear, qNew)) < cmin) {
              qMin = qNear;
              cmin = Cost(qNear) + PathCost(qNear, qNew);
            }
          }
          add(qMin, qNew);

          for (int j = 0; j < 0 || (unsigned) j < Qnear.size(); j++) {
            Node* qNear = Qnear[j];
            if (!obstacles->isSegmentInObstacle(qNew->position, qNear->position) &&
                (Cost(qNew) + PathCost(qNew, qNear)) < Cost(qNear)) {
              Node* qParent = qNear->parent;
              // Remove edge between qParent and qNear
              qParent->children.erase(
                  std::remove(qParent->children.begin(), qParent->children.end(), qNear),
                  qParent->children.end());

              // Add edge between qNew and qNear
              qNear->cost = Cost(qNew) + PathCost(qNew, qNear);
              qNear->parent = qNew;
              qNew->children.push_back(qNear);
            }
          }
        }
      }
    }
    if (reached()) {
      std::cout << "reached" << std::endl;
      setMaxIterations(200);
    }
    // renderArea->update();
    // qApp->processEvents();
  }

  vector<Point> resultPath;
  Node* q;
  if (reached()) {
    q = lastNode;
  } else {
    // if not reached yet, then shortestPath will start from the closest node to end point.
    q = nearest(endPos);
    std::cout << "Exceeded max iterations!" << std::endl;
  }
  // generate shortest path to destination.
  while (q != NULL) {
    resultPath.push_back(q->position);
    path.push_back(q);
    q = q->parent;
  }
  return resultPath;
}
